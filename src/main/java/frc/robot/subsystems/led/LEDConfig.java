package frc.robot.subsystems.led;

import edu.wpi.first.math.util.Units;
import java.lang.Math;

public class LEDConfig {
    public static final int kTurretLEDStripLength = 0;
    public static final int kHopperLEDStripLength = 0;
    public static final int kTotalLEDStripLength = 0;

    public static final double kDisabledRunAnimationLength = 1;
    public static final double kTicksPerAnimationCycle = Units.millisecondsToSeconds(20) * kDisabledRunAnimationLength;
}