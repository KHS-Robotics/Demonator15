package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConfig.DeployerConfig;

public class Deployer extends SubsystemBase {
    // ^ There are two seperate motors for the intake pivot,
    // might want to change right/left names to avoid confusion
    private final RelativeEncoder relativeEncoder;
    private final PIDController pid;
    private final SparkLimitSwitch sensor;
    private final SparkMax motor;

    public Deployer() {

        super(Deployer.class.getSimpleName() + "/" + Deployer.class.getSimpleName());

        var relativeEncoderConfig = new EncoderConfig()
                .positionConversionFactor(DeployerConfig.kDeployerEncoderPositionConversionFactor)
                .velocityConversionFactor(DeployerConfig.kDeployerEncoderVelocityConversionFactor);

        var limitSwitchConfig = new LimitSwitchConfig()
                .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        var deployerLeaderConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(false)
                .apply(limitSwitchConfig)
                .apply(relativeEncoderConfig);
        motor = new SparkMax(RobotMap.INTAKE_DEPLOYER_LEADER_ID, MotorType.kBrushless);
        motor.configure(deployerLeaderConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        relativeEncoder = motor.getEncoder();
        sensor = motor.getForwardLimitSwitch();

        pid = new PIDController(DeployerConfig.kDeployerP, DeployerConfig.kDeployerI, DeployerConfig.kDeployerD);
        pid.setIZone(7);

        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);

    }
}
