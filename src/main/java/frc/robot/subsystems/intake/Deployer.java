package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;

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
  private final RelativeEncoder encoder;
  private final PIDController pid;
  private final SparkLimitSwitch sensor;
  private final SparkMax motor;

  public Deployer() {

    super(Deployer.class.getSimpleName() + "/" + Deployer.class.getSimpleName());
    var encoderConfig = new EncoderConfig()
        .positionConversionFactor(IntakeConfig.DeployerConfig.kDeployerEncoderPositionConversionFactor)
        .velocityConversionFactor(IntakeConfig.DeployerConfig.kDeployerEncoderPositionConversionFactor);

    var limitSwitchConfig = new LimitSwitchConfig()
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    var motorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true)
        .apply(encoderConfig)
        .apply(limitSwitchConfig);
    motor = new SparkMax(RobotMap.INTAKE_DEPLOYER_ID, MotorType.kBrushless);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    sensor = motor.getForwardLimitSwitch();

    pid = new PIDController(DeployerConfig.kDeployerP, DeployerConfig.kDeployerI, DeployerConfig.kDeployerD);
    pid.setIZone(7);

    SmartDashboard.putData(getName(), this);
    SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
  }

  public void periodic() {
    // TODO: re-add once we have reliable absolute encoder or no slop?
    //setMotorOutputForSetpoint();
    //updateSetpointsForDisabledMode();
  }

  public Command setAngleCommand(double angleDegrees) {
    var cmd = this.run(() -> setSetpointAngle(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetDeployerSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopDeployer");
  }

  public Command bangBangControlDeploy() {
    var cmd = startEnd(() -> motor.setVoltage(-3), this::stop);
    return cmd.withTimeout(0.5);
  }

  public Command bangBangControlStow() {
    var cmd = startEnd(() -> motor.setVoltage(2.5), this::stop);
    return cmd.withTimeout(0.67/*six seven */);
  }

  public void setSetpointAngle(double setpointDegrees) {
    // only reset for new setpoints
    if (setpointDegrees != setpointAngleDegrees) {
      pid.reset();
    }
    setpointAngleDegrees = setpointDegrees;
  }

  public Command setStow() {
    var cmd = runEnd(() -> {
      setpointAngleDegrees = IntakeConfig.DeployerSetpoints.STOW;
    }, this::stop);
    return cmd;
  }

    public Command setDeploy() {
    var cmd = runEnd(() -> {
      setpointAngleDegrees = IntakeConfig.DeployerSetpoints.DEPLOY;
    }, this::stop);
    return cmd;
  }

  public double getAngle() {
    return encoder.getPosition() + IntakeConfig.DeployerConfig.kDeployerOffsetAngle;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAngle());
    return (error < 10);
  }

  // if the deployer is at the same spot as the the limitswitch
  public boolean isAtTop() {
    return sensor.isPressed();
  }

  private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getAngle(), setpointAngleDegrees);

    var angle = Math.sin(Math.toRadians(getAngle()));
    var ffGravity = 0;//DeployerConfig.kDeployerKG * angle;

    var output = pidOutput + ffGravity;
    output = MathUtil.clamp(output, -6, 6);
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

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.setSmartDashboardType("Deployer");
      builder.addBooleanProperty("Intake At Setpoint?", () -> this.isAtSetpoint(), null);
      builder.addDoubleProperty("Intake Angle", () -> this.getAngle(), null);
      builder.addDoubleProperty("Intake Setpoint Angle", () -> this.setpointAngleDegrees, null);
  }

}
