"""
AGC Angle Find
=============

A Python implementation of the Apollo Guidance Computer (AGC) angle finding routines.
Based on the original AGC assembly code for Apollo 11's Command Module.

This module handles:
- Computing direction cosine matrices (DCM) between spacecraft and stable member axes
- Calculating optimal rotation axes for spacecraft maneuvers
- Handling gimbal lock conditions
- Converting between different angular representations

Original assembly from: https://github.com/chrislgarry/Apollo-11
"""
import numpy as np
from typing import Dict, List, Optional, Tuple, Union


# Constants from the original code
DP1_4TH = 0.25  # 1/4 in double precision
DPHALF = 0.5    # 1/2 in double precisionz
MINANG = 0.00069375  # Minimum maneuver angle (radians), approx. 0.25 degrees
MAXANG = 0.472222    # Maximum angle before using alternative calculation (radians), approx. 170 degrees

# Gimbal lock constants 
# D = Middle Gimbal Angle (MGA) corresponding to gimbal lock = 60 degrees
# NGL = Buffer angle (to avoid divisions by zero) = 2 degrees
SD = 0.433015    # sin(D), scaled by 2
K3S1 = 0.86603   # sin(D), scaled by 2 
K4 = -0.25       # -cos(D), scaled by 2
K4SQ = 0.125     # cos(D)^2, scaled by 2
SNGLCD = 0.008725  # sin(NGL)cos(D), scaled by 2
CNGL = 0.499695    # cos(NGL), scaled by 2


class AngleFinder:
    """
    Implementation of the AGC angle finding system
    
    This class handles computation of direction cosine matrices, maneuver axes,
    and transformations between different angular representations.
    """
    
    def __init__(self):
        # Initial state vectors and matrices
        self.bcdu = np.zeros(3)    # Current CDU angles
        self.cphi = np.zeros(3)    # Final CDU angles (commanded attitude)
        
        # Transformation matrices
        self.mis = np.eye(3)       # Initial S/C axes to Stable Member axes
        self.mfs = np.eye(3)       # Final S/C axes to Stable Member axes
        self.tmis = np.eye(3)      # Transpose of MIS, scaled by 2
        self.mfi = np.eye(3)       # MFI = TMIS * MFS (scaled by 4)
        self.tmfi = np.eye(3)      # Transpose of MFI, scaled by 4
        self.mfisym = np.eye(3)    # (MFI + TMFI)/2, scaled by 4
        
        # Maneuver parameters
        self.cof = np.zeros(3)     # Maneuver axis
        self.cofskew = np.zeros(3) # Skew-symmetric part of MFI
        self.am = 0.0             # Maneuver angle
        self.cam = 0.0            # Cosine of maneuver angle
        
        # Result matrices
        self.del_matrix = np.eye(3)  # Incremental rotation matrix
    
    def kalcman3(self, current_cdu: np.ndarray, target_cdu: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Calculate maneuver axis and angle to move from current to target attitude
        
        Args:
            current_cdu: Current CDU angles [roll, pitch, yaw] in radians
            target_cdu: Target CDU angles [roll, pitch, yaw] in radians
            
        Returns:
            Tuple of (maneuver_axis, maneuver_angle)
        """
        # Store the initial and target angles
        self.bcdu = current_cdu
        self.cphi = target_cdu
        
        # Compute transformations
        self.mis = self.cdutodcm(self.bcdu)  # Initial S/C axes to Stable Member axes
        self.mfs = self.cdutodcm(self.cphi)  # Final S/C axes to Stable Member axes
        
        # Compute the transformation from initial to final S/C axes
        self.tmis = self.transpos(self.mis)  # Transpose scaled by 2
        self.mfi = self.mxm3(self.tmis, self.mfs)  # MFI = TMIS * MFS (scaled by 4)
        self.tmfi = self.transpos(self.mfi)  # Transpose scaled by 4
        
        # Calculate COFSKEW (skew-symmetric part of MFI)
        self.cofskew = np.array([
            self.tmfi[0, 1] - self.mfi[0, 1],
            self.mfi[0, 2] - self.tmfi[0, 2],
            self.tmfi[1, 2] - self.mfi[1, 2]
        ])
        
        # Calculate AM (maneuver angle)
        self.cam = (self.mfi[0, 0] + self.mfi[1, 1] + self.mfi[2, 2] - 0.25) / 2
        self.am = np.arccos(np.clip(self.cam, -1.0, 1.0))
        
        # Check if maneuver is below minimum threshold
        if self.am < MINANG:
            # Maneuver too small, just return zero angle
            return np.zeros(3), 0.0
        
        # Check if angle is very large
        if self.am > MAXANG:
            # Use alternative calculation for large angles
            self._alt_calc()
        else:
            # Normal case - use COFSKEW as maneuver axis
            self.cof = self.cofskew / np.linalg.norm(self.cofskew)
        
        # Check if maneuver goes through gimbal lock
        if self._check_gimbal_lock():
            # Handle the gimbal lock case
            self._handle_gimbal_lock()
        
        # Find largest component of COF and adjust accordingly
        self._adjust_cof()
        
        return self.cof, self.am
    
    def _alt_calc(self) -> None:
        """Alternative calculation for very large rotation angles"""
        # Calculate the symmetric part of MFI
        self.mfisym = (self.mfi + self.tmfi) / 2
        
        # Calculate COF for the large angle case
        x = np.sqrt(max(0, (self.mfisym[0, 0] - self.cam) / (1 - self.cam)))
        y = np.sqrt(max(0, (self.mfisym[1, 1] - self.cam) / (1 - self.cam)))
        z = np.sqrt(max(0, (self.mfisym[2, 2] - self.cam) / (1 - self.cam)))
        
        self.cof = np.array([x, y, z])
        # Ensure it's a unit vector
        self.cof = self.cof / np.linalg.norm(self.cof)
    
    def _check_gimbal_lock(self) -> bool:
        """Check if maneuver passes through gimbal lock region"""
        # This would implement the LOCSKIRT section in the original code
        # For now just return False - would need to check middle gimbal angle
        return False
    
    def _handle_gimbal_lock(self) -> None:
        """Handle case where maneuver would pass through gimbal lock"""
        # Implement gimbal lock avoidance strategy
        # This would involve calculating a different maneuver path
        pass
    
    def _adjust_cof(self) -> None:
        """
        Determine largest component of COF and adjust accordingly
        
        This implements the COFMAXGO through METHOD3 sections of the original code.
        """
        # Compare magnitudes of components
        if abs(self.cof[0]) >= abs(self.cof[1]) and abs(self.cof[0]) >= abs(self.cof[2]):
            # X component is largest (METHOD1)
            self._method1()
        elif abs(self.cof[1]) >= abs(self.cof[0]) and abs(self.cof[1]) >= abs(self.cof[2]):
            # Y component is largest (METHOD2)
            self._method2()
        else:
            # Z component is largest (METHOD3)
            self._method3()
    
    def _method1(self) -> None:
        """Adjust COF when X component is largest"""
        # Flip whole vector if X component is negative
        if self.cofskew[0] < 0:
            self.cof = -self.cof
        
        # Adjust Y component based on relation to X
        if self.mfisym[0, 1] < 0:  # UX*UY
            self.cof[1] = -self.cof[1]
        
        # Adjust Z component based on relation to X
        if self.mfisym[0, 2] < 0:  # UX*UZ
            self.cof[2] = -self.cof[2]
    
    def _method2(self) -> None:
        """Adjust COF when Y component is largest"""
        # Flip whole vector if Y component is negative
        if self.cofskew[1] < 0:
            self.cof = -self.cof
            
        # Adjust X component based on relation to Y
        if self.mfisym[0, 1] < 0:  # UX*UY
            self.cof[0] = -self.cof[0]
            
        # Adjust Z component based on relation to Y
        if self.mfisym[1, 2] < 0:  # UY*UZ
            self.cof[2] = -self.cof[2]
    
    def _method3(self) -> None:
        """Adjust COF when Z component is largest"""
        # Flip whole vector if Z component is negative
        if self.cofskew[2] < 0:
            self.cof = -self.cof
            
        # Adjust X component based on relation to Z
        if self.mfisym[0, 2] < 0:  # UX*UZ
            self.cof[0] = -self.cof[0]
            
        # Adjust Y component based on relation to Z
        if self.mfisym[1, 2] < 0:  # UY*UZ
            self.cof[1] = -self.cof[1]
    
    def delcomp(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """
        Compute the rotation matrix DEL for a rotation about an axis
        
        Args:
            axis: Unit vector along rotation axis
            angle: Angle of rotation in radians
            
        Returns:
            3x3 rotation matrix
        """
        # Calculate components needed for the rotation matrix
        sin_a = np.sin(angle)
        cos_a = np.cos(angle)
        one_minus_cos = 1.0 - cos_a
        
        # Calculate the components of the DEL matrix
        # DEL = I*cos(A) + uu^T(1-cos(A)) + ux*sin(A)
        
        # Diagonal terms
        xx = axis[0] * axis[0] * one_minus_cos + cos_a
        yy = axis[1] * axis[1] * one_minus_cos + cos_a
        zz = axis[2] * axis[2] * one_minus_cos + cos_a
        
        # Off-diagonal terms
        xy = axis[0] * axis[1] * one_minus_cos
        xz = axis[0] * axis[2] * one_minus_cos
        yz = axis[1] * axis[2] * one_minus_cos
        
        x_sin = axis[0] * sin_a
        y_sin = axis[1] * sin_a
        z_sin = axis[2] * sin_a
        
        # Assemble the rotation matrix
        del_matrix = np.array([
            [xx, xy + z_sin, xz - y_sin],
            [xy - z_sin, yy, yz + x_sin],
            [xz + y_sin, yz - x_sin, zz]
        ])
        
        return del_matrix
    
    def cdutodcm(self, cdu_angles: np.ndarray) -> np.ndarray:
        """
        Convert CDU angles to direction cosine matrix
        
        Args:
            cdu_angles: Array of [roll, pitch, yaw] angles in radians
            
        Returns:
            3x3 direction cosine matrix
        """
        # Extract angles
        phi = cdu_angles[0]    # Roll
        theta = cdu_angles[1]  # Pitch
        psi = cdu_angles[2]    # Yaw
        
        # Compute sines and cosines
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sin_psi = np.sin(psi)
        cos_psi = np.cos(psi)
        
        # Form the direction cosine matrix
        dcm = np.zeros((3, 3))
        
        # First row
        dcm[0, 0] = cos_theta * cos_psi
        dcm[0, 1] = -cos_theta * sin_psi * cos_phi + sin_theta * sin_phi
        dcm[0, 2] = cos_theta * sin_psi * sin_phi + sin_theta * cos_phi
        
        # Second row
        dcm[1, 0] = sin_psi
        dcm[1, 1] = cos_psi * cos_phi
        dcm[1, 2] = -cos_psi * sin_phi
        
        # Third row
        dcm[2, 0] = -sin_theta * cos_psi
        dcm[2, 1] = sin_theta * sin_psi * cos_phi + cos_theta * sin_phi
        dcm[2, 2] = -sin_theta * sin_psi * sin_phi + cos_theta * cos_phi
        
        return dcm
    
    def dcmtocdu(self, dcm: np.ndarray) -> np.ndarray:
        """
        Convert direction cosine matrix to CDU angles
        
        Args:
            dcm: 3x3 direction cosine matrix
            
        Returns:
            Array of [roll, pitch, yaw] angles in radians
        """
        # Extract values from DCM
        # The DCM relates S/C axes to stable member axes
        
        # Calculate middle gimbal angle (PSI)
        psi = np.arcsin(np.clip(dcm[1, 0], -1.0, 1.0))
        cos_psi = np.cos(psi)
        
        # Calculate inner gimbal angle (THETA)
        theta = np.arcsin(np.clip(-dcm[2, 0] / cos_psi, -1.0, 1.0))
        
        # Adjust quadrant for THETA if necessary
        if dcm[0, 0] < 0:
            if theta > 0:
                theta = np.pi - theta
            else:
                theta = -np.pi - theta
        
        # Calculate outer gimbal angle (PHI)
        phi = np.arcsin(np.clip(-dcm[1, 2] / cos_psi, -1.0, 1.0))
        
        # Adjust quadrant for PHI if necessary
        if dcm[1, 1] < 0:
            if phi > 0:
                phi = np.pi - phi
            else:
                phi = -np.pi - phi
        
        return np.array([phi, theta, psi])
    
    def transpos(self, matrix: np.ndarray) -> np.ndarray:
        """
        Transpose a 3x3 matrix
        
        Args:
            matrix: 3x3 input matrix
            
        Returns:
            Transposed matrix
        """
        return matrix.T
    
    def mxm3(self, m1: np.ndarray, m2: np.ndarray) -> np.ndarray:
        """
        Multiply two 3x3 matrices
        
        Args:
            m1: First 3x3 matrix
            m2: Second 3x3 matrix
            
        Returns:
            Product matrix
        """
        return np.matmul(m1, m2)


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians"""
    return degrees * np.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees"""
    return radians * 180.0 / np.pi


if __name__ == "__main__":
    # Simple demonstration
    angle_finder = AngleFinder()
    
    print("=== AGC Angle Find Demonstration ===")
    
    # Current and target attitudes in degrees [roll, pitch, yaw]
    current_attitude_deg = np.array([10.0, 20.0, 30.0])
    target_attitude_deg = np.array([15.0, 25.0, 35.0])
    
    print("Starting attitude (deg):", current_attitude_deg)
    print("Target attitude (deg):", target_attitude_deg)
    
    # Convert to radians
    current_attitude_rad = np.array([degrees_to_radians(x) for x in current_attitude_deg])
    target_attitude_rad = np.array([degrees_to_radians(x) for x in target_attitude_deg])
    
    # Calculate maneuver
    maneuver_axis, maneuver_angle = angle_finder.kalcman3(current_attitude_rad, target_attitude_rad)
    
    print("\nManeuver parameters:")
    print(f"Rotation axis: [{maneuver_axis[0]:.4f}, {maneuver_axis[1]:.4f}, {maneuver_axis[2]:.4f}]")
    print(f"Rotation angle: {radians_to_degrees(maneuver_angle):.2f} degrees")
    
    # Compute the rotation matrix
    rotation_matrix = angle_finder.delcomp(maneuver_axis, maneuver_angle)
    
    print("\nRotation matrix:")
    for row in rotation_matrix:
        print(f"[{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}]") 