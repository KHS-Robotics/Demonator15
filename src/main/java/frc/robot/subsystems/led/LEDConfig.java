package frc.robot.subsystems.led;

import edu.wpi.first.math.util.Units;
import java.lang.Math;

public class LEDConfig {
    public static final int kTurretLEDStripLength = 0;
    public static final int kHopperLEDStripLength = 0;
    public static final int kTotalLEDStripLength = 0;

    public static final float kDisabledRunAnimationLength = 1;
    public static final long kTicksPerAnimationCycle = Math.round(kDisabledRunAnimationLength / Units.millisecondsToSeconds(20));
}