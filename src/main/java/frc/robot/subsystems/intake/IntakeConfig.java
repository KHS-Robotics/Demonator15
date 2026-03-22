package frc.robot.subsystems.intake;

public class IntakeConfig {

    protected final class DeployerSetpoints {
        public static final double STOW = -37.853;
        public static final double AGITATE_HIGH = 0;
        public static final double AGITATE_LOW = 90.0;
        public static final double DEPLOY = 108.287;
        public static final double EXTEND = 90;
    }

    protected final class DeployerConfig {
        public static final double kDeployerP = 0.1;
        public static final double kDeployerI = 0;
        public static final double kDeployerD = 0;

        public static final double kDeployerKG = 1.0313;
        public static final double kDeployerOffsetAngle = -261.654 + 90;

        public static final double kDeployerEncoderPositionConversionFactor = 360.0;
        public static final double kDeployerEncoderVelocityConversionFactor = kDeployerEncoderPositionConversionFactor / 60.0;
    }

    protected final class HopperSetPoints {
        public static final double DEPLOY = 0;
        public static final double STOW = 0;
    }

}
