package frc.robot.subsystems.led;

import edu.wpi.first.math.util.Units;
import java.lang.Math;

public class LEDConfig {
    public static final int kTurretLEDStripLength = 0;
    public static final int kHopperLEDStripLength = 0;
    public static final int kTotalLEDStripLength = 0;

    public static final float kDisabledRunAnimationLength = 2;
    public static final int kTicksToHalfpointAnimationCycle = (int) Math.round((kDisabledRunAnimationLength / Units.millisecondsToSeconds(50)) / 2);
}