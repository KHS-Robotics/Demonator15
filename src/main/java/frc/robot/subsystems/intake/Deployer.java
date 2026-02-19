package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConfig.DeployerConfig;

public class Deployer extends SubsystemBase {

  private double setpointAngleDegrees;
  // ^ There are two seperate motors for the intake pivot,
  // might want to change right/left names to avoid confusion
  private final AbsoluteEncoder encoder;
  private final PIDController pid;
  private final SparkLimitSwitch sensor;
  private final SparkMax motor;

  public Deployer() {

    super(Deployer.class.getSimpleName() + "/" + Deployer.class.getSimpleName());
    var encoderConfig = new AbsoluteEncoderConfig()
        .inverted(true);

    var limitSwitchConfig = new LimitSwitchConfig()
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    var leaderConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .follow(RobotMap.INTAKE_DEPLOYER_LEADER_ID, true)
        .apply(encoderConfig)
        .apply(limitSwitchConfig);
    motor = new SparkMax(RobotMap.INTAKE_DEPLOYER_FOLLOWER_ID, MotorType.kBrushless);
    motor.configure(leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();
    sensor = motor.getForwardLimitSwitch();

    pid = new PIDController(DeployerConfig.kDeployerP, DeployerConfig.kDeployerI, DeployerConfig.kDeployerD);
    pid.setIZone(7);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);

  }

  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }

  public Command stow() {
    var cmd = setAngleCommand(IntakeConfig.DeployerSetpoints.STOW);
    return cmd;
  }

  public Command deploy() {
    var cmd = setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY);
    return cmd;
  }

  public Command agitate() {
    var cmd = setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE);
    return cmd;
  }

  public Command setAngleCommand(double angleDegrees) {
    var cmd = this.run(() -> setSetpointAngle(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetDeployerSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopDeployer");
  }

  public void setSetpointAngle(double setpointDegrees) {
    // only reset for new setpoints
    if (setpointDegrees != setpointAngleDegrees) {
      pid.reset();
    }
    setpointAngleDegrees = setpointDegrees;
  }

  public double getAngle() {
    return Units.rotationsToDegrees(encoder.getPosition()) + DeployerConfig.kDeployerOffsetAngle;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAngle());
    return (error < 2);
  }

  // if the deployer is at the same spot as the the limitswitch
  public boolean isAtTop() {
    return sensor.isPressed();
  }

  private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getAngle(), setpointAngleDegrees);

    var angle = Math.cos(Math.toRadians(getAngle()));
    var ffGravity = DeployerConfig.kDeployerKG * angle;

    var output = pidOutput + ffGravity;
    output = MathUtil.clamp(output, -3, 4);
    motor.setVoltage(output);
  }

  /** Updates the setpoint to the current position. */
  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointAngle(getAngle());
    }
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
    setSetpointAngle(getAngle());
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.addBooleanProperty("IntakeAtTop", () -> this.isAtTop(), null);
  }
}
