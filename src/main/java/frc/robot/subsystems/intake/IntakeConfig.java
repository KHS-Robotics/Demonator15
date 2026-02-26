package frc.robot.subsystems.intake;

public class IntakeConfig {

    protected final class DeployerSetpoints{
        public static final double STOW = 0;
        public static final double AGITATE = 0;
        public static final double DEPLOY = 0;
    }

    protected final class DeployerConfig {
        public static final double kDeployerP = 0;
        public static final double kDeployerI = 0;
        public static final double kDeployerD = 0;

        public static final double kDeployerKG = 0;
        public static final double kDeployerOffsetAngle = 0;
        
        public static final double kDeployerEncoderPositionConversionFactor = 0;
        public static final double kDeployerEncoderVelocityConversionFactor = 0;
    }

    protected final class HopperSetPoints{
        public static final double DEPLOY = 0;
        public static final double STOW = 0;
    }

    protected final class HopperConfig{
        public static final double kDeployerP = 0;
        public static final double kDeployerI = 0;
        public static final double kDeployerD = 0;

        public static final double kHopperEncoderPositionConversionFactor = 1;
        public static final double kHopperEncodervelocityConversionFactor = 1;

    }

}
