package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Time;

public class IntakeConfig {

    protected final class DeployerSetpoints {
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

    protected final class HopperConfig {
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html
        public static final Measure<DimensionlessUnit> kRetractServoPosition = Percent.of(0.0);
        public static final Measure<DimensionlessUnit> kDeployServoPosition = Percent.of(0.5);
        public static final Time kSecondsBetweenStates = Seconds.of(1.0);
    }
}
