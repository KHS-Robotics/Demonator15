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
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
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
    var encoderConfig = new EncoderConfig()
        .positionConversionFactor(IntakeConfig.DeployerConfig.kDeployerEncoderPositionConversionFactor)
        .velocityConversionFactor(IntakeConfig.DeployerConfig.kDeployerEncoderPositionConversionFactor);

    var limitSwitchConfig = new LimitSwitchConfig()
        .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    var absoluteEncoderConfig = new EncoderConfig();

    var motorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30, 10)
        .inverted(true)
        .apply(absoluteEncoderConfig)
        .apply(limitSwitchConfig);
    motor = new SparkMax(RobotMap.INTAKE_DEPLOYER_ID, MotorType.kBrushless);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters,
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

  public Command bangBangControlAgitate(){
    var cmd = startEnd(() -> motor.setVoltage(4.5), this::stop);
    return cmd.withTimeout(0.2);
  }

  public Command bangBangControlUnagitate(){
    var cmd = startEnd(() -> motor.setVoltage(-1.3), this::stop);
    return cmd.withTimeout(0.2);
  }

  public void keepDeployerDown() {
    motor.setVoltage(-1.0);
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

  public double getRotations() {
    return encoder.getPosition();
  }

  public double getAbsoluteAngle(){
    return Units.rotationsToDegrees(getRotations()) + IntakeConfig.DeployerConfig.kDeployerOffsetAngle;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAbsoluteAngle());
    return (error < 10);
  }

  // if the deployer is at the same spot as the the limitswitch
  public boolean isAtTop() {
    return sensor.isPressed();
  }

  private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getAbsoluteAngle(), setpointAngleDegrees);

    var angle = Math.sin(Math.toRadians(getAbsoluteAngle()));
    var ffGravity = 0;//DeployerConfig.kDeployerKG * angle;

    var output = pidOutput + ffGravity;
    output = MathUtil.clamp(output, -3, 3);
    motor.setVoltage(-output);
  }

  /** Updates the setpoint to the current position. */
  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointAngle(getAbsoluteAngle());
    }
  }

  private double getDeployerError() {
    return Math.abs(getAbsoluteAngle() - setpointAngleDegrees);
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
    setSetpointAngle(getAbsoluteAngle());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.setSmartDashboardType("Deployer");
      builder.addBooleanProperty("Intake At Setpoint?", () -> this.isAtSetpoint(), null);
      builder.addDoubleProperty("Deployer Absolute Angle", () -> this.getAbsoluteAngle(), null);
      builder.addDoubleProperty("Intake Setpoint Angle", () -> this.setpointAngleDegrees, null);
      builder.addDoubleProperty("Intake-AppliedOutput", () -> motor.getAppliedOutput(), null);
      builder.addDoubleProperty("error", () -> getDeployerError(), null);
  }

}
