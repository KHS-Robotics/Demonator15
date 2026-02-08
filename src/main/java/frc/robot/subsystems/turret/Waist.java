package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig.HoodConfig;
import frc.robot.subsystems.turret.TurretConfig.WaistConfig;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Waist extends SubsystemBase{
    private final SparkMax motor;
    private final PIDController pid;
    private final AbsoluteEncoder encoder;

    private double setpointRotationDegrees;

    public Waist() {

        var encoderConfig = new AbsoluteEncoderConfig()
                .inverted(true);

        var waistConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(false)
                .apply(encoderConfig);

        motor = new SparkMax(RobotMap.TURRET_AIMER_WAIST_ID, MotorType.kBrushless);
        motor.configure(waistConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        encoder = motor.getAbsoluteEncoder();

        pid = new PIDController(WaistConfig.kAimerWaistP, WaistConfig.kAimerWaistI, WaistConfig.kAimerWaistD);

    }

    public Command setDegreesCommand(double angleDegrees) {
    //0 degrees should be perpendicular with the back
    var cmd = this.run(() -> setSetpointDegrees(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetWaistSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopWaist");
  }

    public double getDegrees() {
    //0 should be perpendicular with the back
    var angle = Units.rotationsToDegrees(encoder.getPosition()) * HoodConfig.kRotationsToDegreesConversion;
    return angle;
  }

    public void setSetpointDegrees(double setpointDegrees) {
    // only reset for new setpoints
    if (setpointDegrees != setpointRotationDegrees) {
      pid.reset();
    }
    setpointRotationDegrees = MathUtil.clamp(setpointDegrees, -180.0, 180.0);
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointRotationDegrees - getDegrees());
    return (error < 2);
  }

    private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getDegrees(), setpointRotationDegrees);
    pidOutput = MathUtil.clamp(pidOutput, -3, 4);
    pidOutput *= TurretConfig.HoodConfig.kRotationsToDegreesConversion;
    motor.setVoltage(pidOutput);
  }

  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointDegrees(getDegrees());
    }
  }

  public void stop() {
    motor.stopMotor();
    pid.reset();
    setSetpointDegrees(getDegrees());
  }

  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }




}
// yaw adjustment
// o–(•o•)–o
//   /   \
//   o   o yay!!!

