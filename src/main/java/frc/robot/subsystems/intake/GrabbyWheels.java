package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConfig.DeployerConfig;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyWheels extends SubsystemBase {
    private final SparkMax motor;
    private final PIDController pid;

    private IntakeState intakeState = IntakeState.IDLE;

    public GrabbyWheels() {

        super(GrabbyWheels.class.getSimpleName() + "/" + GrabbyWheels.class.getSimpleName());

        var GrabbyWheelsConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(false);
        motor = new SparkMax(RobotMap.INTAKE_GRABBY_WHEELS_ID, MotorType.kBrushless);
        motor.configure(GrabbyWheelsConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pid = new PIDController(DeployerConfig.kDeployerP, DeployerConfig.kDeployerI, DeployerConfig.kDeployerD);
        pid.setIZone(7);

        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
    }
    
  public void stop() {
        intakeState = IntakeState.IDLE;
        motor.stopMotor();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopIntake");
  }

  public void intake() {
    intakeState = IntakeState.INTAKING;
    motor.setVoltage(2.75);
  }

  public Command intakeCommand() {
    var cmd = runOnce(this::intake);
    return cmd.withName("StartIntake");
  }

  public void outake() {
    intakeState = IntakeState.OUTTAKING;
    motor.setVoltage(-12);
  }

  public void outakeSlow() {
    intakeState = IntakeState.OUTTAKING;
    motor.setVoltage(-5);
  }

  public Command outakeCommand() {
    var cmd = runOnce(this::outake);
    return cmd.withName("Outtake");
  }

  public Command outakeSlowCommand() {
    var cmd = runOnce(this::outakeSlow);
    return cmd.withName("OuttakeSlow");
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addStringProperty("IntakeState", () -> intakeState.toString(), null);
  }

  public enum IntakeState {
    IDLE("Idle"),
    INTAKING("Intaking"),
    OUTTAKING("Outtaking");

    private final String state;

    private IntakeState(String s) {
      state = s;
    }

    public String toString() {
      return this.state;
    }
  }
}
