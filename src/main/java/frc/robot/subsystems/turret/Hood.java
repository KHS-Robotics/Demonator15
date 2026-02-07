package frc.robot.subsystems.turret;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig.HoodConfig;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hood extends SubsystemBase{
     private final SparkMax motor;
     private final PIDController pid;
     private final AbsoluteEncoder encoder;

     private double setpointAngleDegrees;

     public Hood() {

         var encoderConfig = new AbsoluteEncoderConfig()
           .inverted(true);

         var hoodConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(false)
            .apply(encoderConfig);

         motor = new SparkMax(RobotMap.TURRET_AIMER_HOOD_ID, MotorType.kBrushless);
         motor.configure(hoodConfig, ResetMode.kResetSafeParameters,
             PersistMode.kPersistParameters);

         encoder = motor.getAbsoluteEncoder();

         pid = new PIDController(HoodConfig.kHoodP, HoodConfig.kHoodI, HoodConfig.kHoodD);
     }
     //use angler as base

    public Command setAngleCommand(double angleDegrees) {
    var cmd = this.run(() -> setSetpointAngle(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetHoodSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopHood");
  }



    public double getAngle() {
    //0 is flouiat
    var angle = Units.rotationsToDegrees(encoder.getPosition()) * HoodConfig.kRotationsToDegreesConversion;
    return angle;
  }

    public void setSetpointAngle(double setpointDegrees) {
    // only reset for new setpoints
    if (setpointDegrees != setpointAngleDegrees) {
      pid.reset();
    }
    setpointAngleDegrees = setpointDegrees;
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAngle());
    return (error < 2);
  }

    private void setMotorOutputForSetpoint() {
    var pidOutput = pid.calculate(getAngle(), setpointAngleDegrees);
    pidOutput = MathUtil.clamp(pidOutput, -3, 4);
    pidOutput *= TurretConfig.HoodConfig.kRotationsToDegreesConversion;
    motor.setVoltage(pidOutput);
  }

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

  public void periodic() {
    setMotorOutputForSetpoint();
    updateSetpointsForDisabledMode();
  }

   @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Setpoint", () -> setpointAngleDegrees, this::setSetpointAngle);
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addBooleanProperty("IsAtSetpoint", () -> this.isAtSetpoint(), null);
  }
}



//pitch adjustment 