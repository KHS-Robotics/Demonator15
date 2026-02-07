package frc.robot.subsystems.turret;

public class TurretConfig {

    protected final class HoodConfig {
        public static final double kHoodP = 0;
        public static final double kHoodI = 0;
        public static final double kHoodD = 0;

        public static final double kHoodMaxAngle = 45;
        public static final double kHoodMinAngle = 0;

        //I think we will need a conversion for rotations of motor to how many degrees the hood will change
        public static final double kRotationsToDegreesConversion = 0.0;
    }


    protected final class WaistConfig {
        public static final double kAimerWaistP = 0;
        public static final double kAimerWaistI = 0;
        public static final double kAimerWaistD = 0;
    }
}
