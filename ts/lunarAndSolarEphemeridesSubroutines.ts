/**
 * LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 * ------------------------------------------
 * Purpose: Computes unit position vectors of the Sun and Moon in the basic reference system.
 *
 * Based on the functional description in LUNAR_AND_SOLAR_EPHEMERIDES_SUBROUTINES.agc
 * for Luminary 099.
 */

/**
 * Represents a 3D vector.
 * Components are typically scaled B-1 (range -1 to +1).
 */
export interface Vector3D {
    x: number;
    y: number;
    z: number;
}

/**
 * Astronomical constants required for ephemeris calculations.
 * Values are based on typical values for the Apollo era or derived from similar astronomical models.
 * Angles are in radians. Rates are in radians/day.
 * Time constants are for converting centiseconds to days.
 */
export class EphemeridesConstants {
    // Time conversion
    public static readonly CENTISECONDS_PER_DAY = 8640000; // 24 * 60 * 60 * 100

    // Obliquity of the ecliptic (angle between Earth's equator and ecliptic)
    public static readonly OBLIQUITY_ECLIPTIC = 0.40927970959 // radians (approx 23.4425 degrees)
    public static readonly COS_OBLIQUITY = Math.cos(EphemeridesConstants.OBLIQUITY_ECLIPTIC);
    public static readonly SIN_OBLIQUITY = Math.sin(EphemeridesConstants.OBLIQUITY_ECLIPTIC);

    // Inclination of Moon's orbit to the ecliptic
    public static readonly LUNAR_ORBIT_INCLINATION = 0.08979719 // radians (approx 5.145396 degrees)
    public static readonly SIN_LUNAR_INCLINATION = Math.sin(EphemeridesConstants.LUNAR_ORBIT_INCLINATION);

    // Moon vector transformation constants (derived from OBLIQUITY and LUNAR_ORBIT_INCLINATION)
    // K1 = COS(OBL)
    public static readonly K1_MOON = EphemeridesConstants.COS_OBLIQUITY;
    // K2 = SIN(OBL)*SIN(IM)
    public static readonly K2_MOON = EphemeridesConstants.SIN_OBLIQUITY * EphemeridesConstants.SIN_LUNAR_INCLINATION;
    // K3 = SIN(OBL)
    public static readonly K3_MOON = EphemeridesConstants.SIN_OBLIQUITY;
    // K4 = COS(OBL)*SIN(IM)
    public static readonly K4_MOON = EphemeridesConstants.COS_OBLIQUITY * EphemeridesConstants.SIN_LUNAR_INCLINATION;

    // Sun's longitude parameters (LOS)
    public static readonly LOS0 = 1.7279594_72 // Longitude of Sun at epoch (radians, approx 98.99 deg for mid-1969)
    public static readonly LOSR = 0.017202124_2 // Mean rate for Sun (radians/day, approx 0.9856 deg/day)
    public static readonly CMOD_SUN = 0.0334882_2 // Fudge factor C (radians, approx 1.9186 deg)
    public static readonly CARG_SUN = 6.2221576_1 // Argument for Sun's correction (radians, approx 356.522 deg)
    // Rate for Sun's correction term: (2*PI)/365.24 days (mean tropical year)
    public static readonly CRATE_CONST_SUN = (2 * Math.PI) / 365.24219879;

    // Moon's mean longitude parameters (LOM)
    public static readonly LOM0 = 0.6535786_2 // Mean longitude of Moon at epoch (radians)
    public static readonly LOMR = 0.2280271_36 // Mean rate for Moon's longitude (radians/day)
    // Correction term A for LOM
    public static readonly AMOD_MOON = 0.0109762_3 // Fudge factor A (radians)
    public static readonly AARG_MOON = 0.2322222_2 // Argument for A term (radians)
    // Rate for A term: (2*PI)/27.5545 days (anomalistic month based on description, though 27.32 is sidereal)
    public static readonly ARATE_CONST_MOON = (2 * Math.PI) / 27.5545;
    // Correction term B for LOM
    public static readonly BMOD_MOON = 0.0022239_2 // Fudge factor B (radians)
    public static readonly BARG_MOON = 0.5672222_2 // Argument for B term (radians)
    // Rate for B term: (2*PI)/32 days (seems like an empirical period from AGC notes)
    public static readonly BRATE_CONST_MOON = (2 * Math.PI) / 32.0;

    // Moon's longitude of node parameters (LON)
    public static readonly LON0 = 2.1234567 // Longitude of Moon's node at epoch (radians) - example value
    public static readonly LONR = -0.0000925 // Mean rate for Moon's node (radians/day, retrograde motion) - example value
}

export class EphemeridesSubroutines {
    private constants: EphemeridesConstants;

    constructor() {
        // In a real scenario, constants might be versioned or dynamically loaded for the specific mission year.
        // Here, we use the static class.
        this.constants = new EphemeridesConstants(); // Instance not strictly needed if all static
    }

    /**
     * Computes unit position vector of the Sun and Moon.
     * @param timeSinceLaunchCS Time since launch in centiseconds.
     * @param tephemCS Time from reference epoch (midnight 1 July preceding launch) to launch, in centiseconds.
     * @returns An object containing the unit position vector of the Sun (vSun) and Moon (vMoon).
     */
    public locateSunAndMoon(
        timeSinceLaunchCS: number,
        tephemCS: number
    ): { vSun: Vector3D; vMoon: Vector3D } {
        
        // Total time from reference epoch in centiseconds
        const totalTimeCS = timeSinceLaunchCS + tephemCS;
        
        // Convert total time to days
        const timeP = totalTimeCS / EphemeridesConstants.CENTISECONDS_PER_DAY;

        // --- Calculate Sun's position (VSUN) ---
        // LOS = LOS0 + LOSR*T - (CMOD * SIN(CRATE_CONST*T + CARG))
        const sunCorrectionTermAngle = EphemeridesConstants.CRATE_CONST_SUN * timeP + EphemeridesConstants.CARG_SUN;
        const sunCorrection = EphemeridesConstants.CMOD_SUN * Math.sin(sunCorrectionTermAngle);
        const los = EphemeridesConstants.LOS0 + EphemeridesConstants.LOSR * timeP - sunCorrection;

        const cosLos = Math.cos(los);
        const sinLos = Math.sin(los);

        const vSun: Vector3D = {
            x: cosLos,
            y: EphemeridesConstants.COS_OBLIQUITY * sinLos,
            z: EphemeridesConstants.SIN_OBLIQUITY * sinLos,
        };

        // --- Calculate Moon's position (VMOON) ---
        // LOM = LOM0 + LOMR*T - (AMOD*SIN(ARATE_CONST*T + AARG) + BMOD*SIN(BRATE_CONST*T + BARG))
        const moonCorrectionTermAAngle = EphemeridesConstants.ARATE_CONST_MOON * timeP + EphemeridesConstants.AARG_MOON;
        const moonCorrectionA = EphemeridesConstants.AMOD_MOON * Math.sin(moonCorrectionTermAAngle);

        const moonCorrectionTermBAngle = EphemeridesConstants.BRATE_CONST_MOON * timeP + EphemeridesConstants.BARG_MOON;
        const moonCorrectionB = EphemeridesConstants.BMOD_MOON * Math.sin(moonCorrectionTermBAngle);
        
        const lom = EphemeridesConstants.LOM0 + EphemeridesConstants.LOMR * timeP - (moonCorrectionA + moonCorrectionB);

        // LON = LON0 + LONR*T
        const lon = EphemeridesConstants.LON0 + EphemeridesConstants.LONR * timeP;

        const cosLom = Math.cos(lom);
        const sinLom = Math.sin(lom);
        const sinLomMinusLon = Math.sin(lom - lon);

        // Using K constants as derived for the transformation matrix:
        // M = (COS(LOM), 
        //      COS(OBL)*SIN(LOM) - SIN(OBL)*SIN(IM)*SIN(LOM-LON), 
        //      SIN(OBL)*SIN(LOM) + COS(OBL)*SIN(IM)*SIN(LOM-LON))
        // Which translates to:
        // M = (cosLom,
        //      K1*sinLom - K2*sin(lom-lon),
        //      K3*sinLom + K4*sin(lom-lon) )
        const vMoon: Vector3D = {
            x: cosLom,
            y: EphemeridesConstants.K1_MOON * sinLom - EphemeridesConstants.K2_MOON * sinLomMinusLon,
            z: EphemeridesConstants.K3_MOON * sinLom + EphemeridesConstants.K4_MOON * sinLomMinusLon,
        };

        return { vSun, vMoon };
    }
}

// Example Usage:
function exampleUsageLspos(): void {
    const ephemerides = new EphemeridesSubroutines();

    // Example time values (in centiseconds)
    // TEPHEM: Time from midnight 1 July preceding launch to launch.
    // Let's assume launch is July 16, 1969. TEPHEM would be for 15 days.
    const tephemCS_example = 15 * EphemeridesConstants.CENTISECONDS_PER_DAY; 
    
    // Time since launch: e.g., 3 days into the mission
    const timeSinceLaunchCS_example = 3 * EphemeridesConstants.CENTISECONDS_PER_DAY;

    const { vSun, vMoon } = ephemerides.locateSunAndMoon(timeSinceLaunchCS_example, tephemCS_example);

    console.log("Sun Position (VSUN):");
    console.log(`  x: ${vSun.x.toFixed(8)}`);
    console.log(`  y: ${vSun.y.toFixed(8)}`);
    console.log(`  z: ${vSun.z.toFixed(8)}`);

    console.log("\nMoon Position (VMOON):");
    console.log(`  x: ${vMoon.x.toFixed(8)}`);
    console.log(`  y: ${vMoon.y.toFixed(8)}`);
    console.log(`  z: ${vMoon.z.toFixed(8)}`);

    // For verification, these unit vectors should have a magnitude close to 1.
    const magSun = Math.sqrt(vSun.x**2 + vSun.y**2 + vSun.z**2);
    const magMoon = Math.sqrt(vMoon.x**2 + vMoon.y**2 + vMoon.z**2);
    console.log(`\nMagnitude of VSUN: ${magSun.toFixed(8)} (should be close to 1)`);
    console.log(`Magnitude of VMOON: ${magMoon.toFixed(8)} (should be close to 1)`);
}

// To run this example if this file is executed directly:
// exampleUsageLspos(); 