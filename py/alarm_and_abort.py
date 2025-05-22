"""
AGC Alarm and Abort Systems
===========================

A Python implementation of the Apollo Guidance Computer (AGC) alarm and abort system.
Based on the original AGC assembly code for Apollo 11's Command Module.

The alarm system allows the AGC to report non-fatal error conditions by:
- Turning on the program alarm light
- Optionally displaying alarm codes
- Handling different severity levels (alarms, aborts, etc.)

Original assembly from: https://github.com/chrislgarry/Apollo-11
"""
import enum
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Union, Callable


# Bit patterns from original code
BIT15 = 0o040000
OCT40400 = 0o40400
OCT77770 = 0o77770
OCT14 = 0o14
OCT1103 = 0o1103
OCT217 = 0o00217
POSMAX = 0o37777  # Positive maximum (2^15-1)


class AlarmType(enum.Enum):
    """Types of alarms in the AGC system"""
    ALARM = "ALARM"           # Non-abortive alarm, continues program
    BAILOUT = "BAILOUT"       # Serious error, program may continue
    POODOO = "POODOO"         # Serious error requiring program restart
    VARALARM = "VARALARM"     # Variable alarm (user-defined code)
    PRIOLARM = "PRIOLARM"     # Priority alarm that displays and waits for user
    CCSHOLE = "CCSHOLE"       # CCS instructions executed hole pattern
    CURTAINS = "CURTAINS"     # Fatal error


class RestartType(enum.Enum):
    """Types of restart/recovery actions"""
    NONE = "NONE"                 # No restart required
    BAILOUT = "BAILOUT"           # Program "bails out" but continues
    WHIMPER = "WHIMPER"           # Quiet restart of some subsystems
    ENEMA = "ENEMA"               # More aggressive reset
    MR_KLEAN = "MR_KLEAN"         # Full system clean


class DSKYState:
    """
    Simulates the Display and Keyboard (DSKY) state
    
    This includes registers for displaying information and
    status indicators like alarm lights.
    """
    
    def __init__(self):
        # Display registers
        self.noun = 0
        self.verb = 0
        self.r1 = 0  # Register 1
        self.r2 = 0  # Register 2
        self.r3 = 0  # Register 3
        
        # Status indicators
        self.prog_alarm_light = False
        self.restart_light = False
        self.standby_light = False
        self.key_rel_light = False
        self.opr_err_light = False
        self.temp_light = False
        self.gimbal_lock_light = False
        self.no_att_light = False
        self.vel_light = False
        self.alt_light = False
        
        # Display table (represents internal display state)
        self.dsptab = [0] * 12  # DSPTAB in original code
        
        # Wait state
        self.waiting_for_response = False
    
    def set_display(self, verb: int, noun: int, r1: int = None, r2: int = None, r3: int = None):
        """Set the main display registers"""
        self.verb = verb
        self.noun = noun
        if r1 is not None:
            self.r1 = r1
        if r2 is not None:
            self.r2 = r2
        if r3 is not None:
            self.r3 = r3
        
        # Update display table as needed
        # In the original code, these would be mapped to specific segments
        
    def turn_on_prog_alarm(self):
        """Turn on the program alarm light"""
        self.prog_alarm_light = True
        
        # Simulate updating DSPTAB as in original code
        self.dsptab[11] = self.dsptab[11] | OCT40400
    
    def turn_off_prog_alarm(self):
        """Turn off the program alarm light"""
        self.prog_alarm_light = False
        
        # Simulate updating DSPTAB as would happen in original
        self.dsptab[11] = self.dsptab[11] & ~OCT40400


@dataclass
class AGCState:
    """
    Represents the current state of essential AGC components needed for alarm handling
    
    This includes registers, flags, and other state information that
    alarm and abort routines need to access or modify.
    """
    # General registers
    a: int = 0           # Accumulator
    l: int = 0           # Lower accumulator
    q: int = 0           # Q register (return address)
    z: int = 0           # Program counter
    bbank: int = 0       # Basic bank register
    brupt: int = 0       # Interrupt return address
    
    # Special registers
    superbnk: int = 0    # Super bank bits
    failreg: List[int] = None  # Failure registers
    itemp1: int = 0      # Temporary storage
    almcadr: List[int] = None  # Alarm caller address
    
    # Program state
    loc: int = 0         # Location counter
    bankset: int = 0     # Current bank setting
    
    # Flags
    stateflg: bool = False    # State flag
    reintflg: bool = False    # Reintegration flag
    nodoflag: bool = False    # No-do flag
    v37flbit: bool = False    # V37 flag bit (average G on/off)
    flagwrd7: int = 0         # Flag word 7 container
    
    # Other state
    buf2: List[int] = None    # Buffer for 2CADR storage
    localarm: int = 0         # Local alarm data
    bankalrm: int = 0         # Bank alarm data
    dsky: DSKYState = None    # DSKY state
    
    def __post_init__(self):
        """Initialize default values for complex types"""
        if self.failreg is None:
            self.failreg = [0, 0, 0]  # 3 failure registers
        if self.almcadr is None:
            self.almcadr = [0, 0]     # 2-word alarm caller address
        if self.buf2 is None:
            self.buf2 = [0, 0]        # 2-word buffer
        if self.dsky is None:
            self.dsky = DSKYState()


class AGCAlarmSystem:
    """
    Implementation of the AGC Alarm and Abort system
    
    This handles various types of alarm conditions in the AGC,
    from minor alarms to serious aborts that require program restarts.
    """
    
    def __init__(self):
        self.state = AGCState()
        self.logger = logging.getLogger("AGCAlarmSystem")
        
        # For demonstration/testing
        self.last_alarm = None
        self.last_alarm_code = None
        
        # Constants
        self.v05n09 = 0o0509  # VERB 05 NOUN 09 - display alarm codes
    
    def inhint(self):
        """Inhibit interrupts (simulated)"""
        # In a real simulation we would disable interrupts
        # Here we just log it
        self.logger.debug("INHINT - Interrupts inhibited")
    
    def relint(self):
        """Release interrupt inhibition (simulated)"""
        # In a real simulation we would re-enable interrupts
        # Here we just log it
        self.logger.debug("RELINT - Interrupts enabled")
    
    def alarm(self, alarm_code: int) -> None:
        """
        Non-abortive alarm that turns on the program alarm light
        
        Args:
            alarm_code: The alarm code to register
        """
        self.inhint()
        
        # Save caller's return address (Q register)
        caller_address = self.state.q
        self.state.almcadr = caller_address
        
        # Get alarm code (originally done via indexing Q)
        self.state.l = alarm_code
        
        # Process the alarm through common entry point
        self._bortent(alarm_code)
        
        # Return to caller
        return caller_address + 1
    
    def _bortent(self, alarm_code: int) -> None:
        """
        Common entry point for alarms and aborts
        
        Args:
            alarm_code: The alarm or abort code
        """
        self.state.l = alarm_code
        
        # Continue to priority processing
        self._prioent()
    
    def _prioent(self) -> None:
        """Priority processing for alarms"""
        # Save current bank + superbank bits
        bank_with_super = self.state.bbank
        bank_with_super = ((bank_with_super << 1) & 0o177776) | ((self.state.superbnk >> 9) & 0o1)
        self.state.almcadr[1] = bank_with_super
        
        # Continue to main alarm processing
        self._larment()
    
    def _larment(self) -> None:
        """Main alarm processing logic"""
        # Store return address
        self.state.itemp1 = self.state.q
        
        # Store current location and bank
        self.state.localarm = self.state.loc
        self.state.bankalrm = self.state.bankset
        
        # Check failure registers
        self._chkfail1()
    
    def _chkfail1(self) -> None:
        """Check first failure register"""
        if self.state.failreg[0] != 0:
            # Something already in FAILREG
            self._chkfail2()
        else:
            # Store alarm code in first failure register
            self.state.failreg[0] = self.state.l
            # Turn on alarm light for first alarm
            self._proglarm()
    
    def _chkfail2(self) -> None:
        """Check second failure register"""
        if self.state.failreg[1] != 0:
            # Move to check third register
            self._fail3()
        else:
            # Store alarm code in second failure register
            self.state.failreg[1] = self.state.l
            # Multiple alarms, exit
            self._multexit()
    
    def _fail3(self) -> None:
        """Check third failure register"""
        # Check if register is full
        masked_value = self.state.failreg[2] & POSMAX
        if masked_value != 0:
            # All three registers full, handle multiple failure
            self._multfail()
        else:
            # Store in third register
            self.state.failreg[2] = self.state.l
            # Multiple alarms, exit
            self._multexit()
    
    def _proglarm(self) -> None:
        """Turn on program alarm light"""
        # Update DSKY display table to turn on alarm light
        self.state.dsky.turn_on_prog_alarm()
        
        # Exit after setting alarm
        self._multexit()
    
    def _multexit(self) -> None:
        """Exit after handling multiple alarms"""
        # Get return address
        ret_addr = self.state.itemp1
        
        # Re-enable interrupts
        self.relint()
        
        # Return to caller (simulated)
        self.logger.debug(f"MULTEXIT - Returning to address {ret_addr:o}")
        return ret_addr
    
    def _multfail(self) -> None:
        """Handle case of multiple failures filling registers"""
        # Take current alarm code and combine with bit 15 set
        new_value = self.state.l + BIT15
        # Store in third register
        self.state.failreg[2] = new_value
        
        # Exit
        self._multexit()
    
    def priolarm(self, alarm_code: int) -> Union[int, List[int]]:
        """
        Priority alarm that displays and waits for user response
        
        Args:
            alarm_code: The alarm code to display
            
        Returns:
            Either single return address or list of possible returns
            depending on implementation needs
        """
        self.inhint()
        
        # Save alarm code
        self.state.l = alarm_code
        
        # Save caller's address
        self.state.almcadr = self.state.buf2[0]
        bank_with_super = self.state.buf2[1]
        
        # Continue to priority processing
        self._prioent()
        
        # In the original, this would display VERB 5 NOUN 9
        # and wait for astronaut response
        self.state.dsky.set_display(5, 9, r1=alarm_code)
        self.state.dsky.waiting_for_response = True
        
        # For our simulation, return possible return addresses
        # In the real AGC, only one would be taken depending on user action
        return [
            self.state.almcadr + 1,  # Astronaut return 1
            self.state.almcadr + 2,  # Astronaut return 2
            self.state.almcadr + 3,  # Astronaut return 3
            self.state.almcadr + 4   # Immediate return
        ]
    
    def bailout(self, alarm_code: int) -> None:
        """
        Serious error handling that stores state for debugging
        
        Args:
            alarm_code: The alarm code
        """
        self.inhint()
        
        # Save caller's return address
        self.state.almcadr = self.state.q
        
        # Call VAC5STOR to store erasables for debugging
        self._vac5stor()
        
        # Get alarm code and process through common entry
        self._bortent(alarm_code)
        
        # Check if average G is on
        if self.state.v37flbit & self.state.flagwrd7:
            # If so, don't do full POODOO, do BAILOUT instead
            return self._whimper()
        
        # Turn off various flags
        self.state.stateflg = False
        self.state.reintflg = False
        self.state.nodoflag = False
        
        # Call MR.KLEAN for system cleanup
        self._mr_klean()
        
        # Continue to WHIMPER
        return self._whimper()
    
    def _vac5stor(self) -> None:
        """Store erasables for debugging purposes"""
        # This would save key system state for debugging
        # Simplified implementation just logs the action
        self.logger.debug("VAC5STOR - Storing erasable memory for debugging")
    
    def _whimper(self) -> None:
        """Handle quiet restart"""
        self.inhint()
        
        # Set up BRUPT register for restart
        self.state.brupt = 2 + self.state.z
        
        # In the original, this would call RESUME then go to ENEMA
        # Simulate that sequence
        self.logger.info("WHIMPER - Performing quiet restart sequence")
        
        # Call ENEMA for system reset
        return self._enema()
    
    def _enema(self) -> None:
        """Perform system reset"""
        # This would reset various subsystems
        # Simplified implementation just logs the action
        self.logger.info("ENEMA - Performing system reset")
    
    def _mr_klean(self) -> None:
        """Perform full system cleanup"""
        # This would do a more thorough system cleanup
        # Simplified implementation just logs the action
        self.logger.info("MR.KLEAN - Performing full system cleanup")
    
    def poodoo(self, alarm_code: int) -> None:
        """
        Critical abort handling
        
        Args:
            alarm_code: The abort code
        """
        self.inhint()
        
        # Save caller's return address
        self.state.almcadr = self.state.q
        
        # Store erasables for debugging
        self._vac5stor()
        
        # Get alarm code and continue to abort processing
        return self._abort2(alarm_code)
    
    def _abort2(self, alarm_code: int) -> None:
        """Common abort processing"""
        # Process the alarm code
        self._bortent(alarm_code)
        
        # Continue with abort sequence (same as BAILOUT)
        # In a real implementation, this would follow the same path
        # as the BAILOUT function above
        return self.bailout(alarm_code)
    
    def ccshole(self) -> None:
        """Handle CCS hole pattern error"""
        self.inhint()
        
        # Save caller's return address
        self.state.almcadr = self.state.q
        
        # Store erasables for debugging
        self._vac5stor()
        
        # Process with specific alarm code
        return self._abort2(OCT1103)
    
    def curtains(self, alarm_code: int) -> int:
        """
        Handle fatal error
        
        Args:
            alarm_code: The error code
            
        Returns:
            Return address
        """
        self.inhint()
        
        # Save caller's Q
        self.state.almcadr = self.state.q
        
        # Process alarm
        self._alarm2(OCT217 if alarm_code is None else alarm_code)
        
        # Return to caller
        return self.state.almcadr
    
    def _alarm2(self, alarm_code: int) -> None:
        """Secondary alarm entry point"""
        self.state.almcadr = self.state.q
        
        # Get alarm code and process
        self._bortent(alarm_code)
    
    def varalarm(self, alarm_code: int) -> int:
        """
        Variable alarm that turns on light but doesn't display
        
        Args:
            alarm_code: User-defined alarm code
            
        Returns:
            Return address
        """
        self.inhint()
        
        # Save alarm code
        self.state.l = alarm_code
        
        # Save caller's return address
        self.state.almcadr = self.state.q
        
        # Process through priority entry point
        self._prioent()
        
        # Return to caller
        return self.state.almcadr
    
    def trigger_alarm(self, alarm_type: AlarmType, alarm_code: int) -> Dict:
        """
        Public method to trigger different types of alarms
        
        Args:
            alarm_type: The type of alarm to trigger
            alarm_code: The alarm code
            
        Returns:
            Dictionary with alarm results
        """
        self.last_alarm = alarm_type
        self.last_alarm_code = alarm_code
        
        # Call appropriate handler based on alarm type
        if alarm_type == AlarmType.ALARM:
            result = self.alarm(alarm_code)
        elif alarm_type == AlarmType.BAILOUT:
            result = self.bailout(alarm_code)
        elif alarm_type == AlarmType.POODOO:
            result = self.poodoo(alarm_code)
        elif alarm_type == AlarmType.VARALARM:
            result = self.varalarm(alarm_code)
        elif alarm_type == AlarmType.PRIOLARM:
            result = self.priolarm(alarm_code)
        elif alarm_type == AlarmType.CCSHOLE:
            result = self.ccshole()
        elif alarm_type == AlarmType.CURTAINS:
            result = self.curtains(alarm_code)
        else:
            raise ValueError(f"Unknown alarm type: {alarm_type}")
        
        # Return alarm status
        return {
            "alarm_type": alarm_type,
            "alarm_code": alarm_code,
            "failreg": self.state.failreg,
            "result": result,
            "prog_alarm_light": self.state.dsky.prog_alarm_light,
            "waiting_for_response": self.state.dsky.waiting_for_response
        }
    
    def get_alarm_status(self) -> Dict:
        """
        Get current alarm system status
        
        Returns:
            Dictionary with current alarm state
        """
        return {
            "failreg": self.state.failreg,
            "prog_alarm_light": self.state.dsky.prog_alarm_light,
            "last_alarm": self.last_alarm,
            "last_alarm_code": self.last_alarm_code,
            "dsky_state": {
                "verb": self.state.dsky.verb,
                "noun": self.state.dsky.noun,
                "r1": self.state.dsky.r1,
                "r2": self.state.dsky.r2,
                "r3": self.state.dsky.r3,
                "waiting_for_response": self.state.dsky.waiting_for_response
            }
        }


# For backward compatibility with the original assembly labels
ALARM = AlarmType.ALARM
BAILOUT = AlarmType.BAILOUT
POODOO = AlarmType.POODOO
VARALARM = AlarmType.VARALARM
PRIOLARM = AlarmType.PRIOLARM
CCSHOLE = AlarmType.CCSHOLE
CURTAINS = AlarmType.CURTAINS


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )
    
    # Create alarm system and demonstrate usage
    alarm_system = AGCAlarmSystem()
    
    print("=== AGC Alarm System Demonstration ===")
    
    # Trigger a basic alarm
    print("\n1. Triggering basic alarm (code 1201)...")
    result = alarm_system.trigger_alarm(AlarmType.ALARM, 0o1201)
    print(f"   Alarm registered: {result['alarm_type'].value}, code: {result['alarm_code']:o}")
    print(f"   Program alarm light: {'ON' if result['prog_alarm_light'] else 'OFF'}")
    
    # Show failure registers
    print(f"   Failure registers: {[f'{reg:o}' for reg in result['failreg']]}")
    
    # Trigger a priority alarm
    print("\n2. Triggering priority alarm (code 1202)...")
    result = alarm_system.trigger_alarm(AlarmType.PRIOLARM, 0o1202)
    print(f"   Alarm registered: {result['alarm_type'].value}, code: {result['alarm_code']:o}")
    print(f"   Display showing: VERB {alarm_system.state.dsky.verb:02d} NOUN {alarm_system.state.dsky.noun:02d}")
    print(f"   R1: {alarm_system.state.dsky.r1:o} (alarm code)")
    print(f"   Waiting for astronaut response: {result['waiting_for_response']}")
    
    # Trigger a severe error
    print("\n3. Triggering POODOO abort (code 22000)...")
    result = alarm_system.trigger_alarm(AlarmType.POODOO, 0o22000)
    print(f"   Abort sequence initiated, code: {result['alarm_code']:o}")
    
    print("\nAlarm system final status:")
    status = alarm_system.get_alarm_status()
    print(f"   Last alarm: {status['last_alarm'].value}, code: {status['last_alarm_code']:o}")
    print(f"   Failure registers: {[f'{reg:o}' for reg in status['failreg']]}")
    print(f"   Program alarm light: {'ON' if status['prog_alarm_light'] else 'OFF'}") 