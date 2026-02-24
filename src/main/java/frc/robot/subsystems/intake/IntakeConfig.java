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
        public static final double HOPPERRETRACT = 0;
        public static final double HOPPERDEPLOY = 0;
        public static final double HOPPERSTOW = 0;
    }

}
