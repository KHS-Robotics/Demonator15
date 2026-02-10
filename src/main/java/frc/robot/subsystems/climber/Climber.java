package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.climber.ClimberConfig.ClimberHookAngle;
import frc.robot.subsystems.climber.ClimberConfig.ClimberMotorConfig;
import frc.robot.subsystems.climber.ClimberConfig.ClimberSetpoints;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

public class Climber extends SubsystemBase {
    private final SparkMax motor;
    private final PIDController pid;
    private final SparkLimitSwitch bottomLimitSwitch;
    private final RelativeEncoder relativeEncoder;
    private final SparkAnalogSensor absoluteEncoder;
    private final Servo outerServoLeft, outerServoRight, innerServo;
    private double setpointHeightFromGroundInches = ClimberSetpoints.STOW;

    private enum ClimberHeightMode {
        kRelative, kAbsolute;
    }

    private ClimberHeightMode heightMode = ClimberHeightMode.kRelative;

    public Climber() {

        super(Climber.class.getSimpleName() + "/" + Climber.class.getSimpleName());

        // var relativeEncoderConfig = new EncoderConfig()
        // .positionConversionFactor(ClimberConfig.kElevatorEncoderPositionConversionFactor)
        // .velocityConversionFactor(ClimberConfig.kElevatorEncoderVelocityConversionFactor);

        var limitSwitchConfig = new LimitSwitchConfig()
                .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)
                .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(Type.kNormallyClosed);

        var motorConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(false)
                // .apply(relativeEncoderConfig)
                .apply(limitSwitchConfig);
        motor = new SparkMax(RobotMap.CLIMBER_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        relativeEncoder = motor.getEncoder();

        absoluteEncoder = motor.getAnalog();

        bottomLimitSwitch = motor.getReverseLimitSwitch();

        pid = new PIDController(ClimberMotorConfig.kClimberP, ClimberMotorConfig.kClimberI,
                ClimberMotorConfig.kClimberD);
        pid.setIZone(3);

        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);

        setSetpointHeight(getHeightFromGroundInches());

        outerServoLeft = new Servo(1);
        outerServoRight = new Servo(2);
        innerServo = new Servo(3);
    }

    public void periodic() {
        setMotorOutputForSetpoint();
        updateSetpointsForDisabledMode();
    }

    private void updateSetpointsForDisabledMode() {
        if (RobotState.isDisabled()) {
            setSetpointHeight(getHeightFromGroundInches());
        }
    }

    public boolean isAtSetpoint() {
        var error = Math.abs(setpointHeightFromGroundInches - getHeightFromGroundInches());
        return (error < 0.5);
    }

    public Command setHeightCommand(double heightFromGround) {
        var cmd = this.run(() -> setSetpointHeight(heightFromGround)).until(this::isAtSetpoint);
        return cmd.withName("SetClimberSetHeightPoint");
    }

    public void stop() {
        motor.stopMotor();
        pid.reset();
        setSetpointHeight(getHeightFromGroundInches());
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        return cmd.withName("StopClimber");
    }

    public void deployOuterHooks() {
        outerServoLeft.setAngle(ClimberHookAngle.DeployedHookAngle);
        outerServoRight.setAngle(ClimberHookAngle.DeployedHookAngle);
    }

    public void deployInnerHooks() {
        innerServo.setAngle(ClimberHookAngle.DeployedHookAngle);
    }

    public void retractOuterhooks() {
        outerServoLeft.setAngle(ClimberHookAngle.RetractedHookAngle);
        outerServoRight.setAngle(ClimberHookAngle.RetractedHookAngle);
    }

    public void retractInnerHook() {
        innerServo.setAngle(ClimberHookAngle.RetractedHookAngle);
    }

    public void STOWAllHooks() {
        outerServoLeft.setAngle(ClimberHookAngle.HookAngleSTOW);
        outerServoRight.setAngle(ClimberHookAngle.HookAngleSTOW);
        innerServo.setAngle(ClimberHookAngle.HookAngleSTOW);
    }

    private void setMotorOutputForSetpoint() {
        var pidOutput = pid.calculate(getHeightFromGroundInches(), setpointHeightFromGroundInches);
        var output = pidOutput + ClimberMotorConfig.kClimberKG;

        if (isAtSetpoint() && heightMode == ClimberHeightMode.kAbsolute) {
            pid.reset();
            output = ClimberMotorConfig.kClimberKG;
        }
    }

    public void setSetpointHeight(double heightFromGroundInches) {
        // extra precaution to prevent too low of setpoints
        if (heightFromGroundInches < ClimberSetpoints.STOW) {
            heightFromGroundInches = ClimberSetpoints.STOW;
        }
        // extra precaution to prevent too large of setpoints
        if (heightFromGroundInches > ClimberSetpoints.L3) {
            heightFromGroundInches = ClimberSetpoints.L3;
        }

        // only reset for new setpoints
        if (setpointHeightFromGroundInches != heightFromGroundInches) {
            pid.reset();
        }
        setpointHeightFromGroundInches = heightFromGroundInches;
    }

    private double getPotentiometerVoltage() {
        return absoluteEncoder.getVoltage();
    }

    public double getHeightFromGroundInchesUsingAbsoluteEncoder() {
        var deltaHeightInches = ClimberSetpoints.L3 - ClimberSetpoints.STOW;
        var relativeVoltage = getPotentiometerVoltage() - ClimberConfig.kClimberAbsoluteEncoderMinVoltage;
        var deltaVoltage = ClimberConfig.kClimberAbsoluteEncoderMaxVoltage
                - ClimberConfig.kClimberAbsoluteEncoderMinVoltage;
        var heightFromStowInches = (deltaHeightInches * relativeVoltage) / deltaVoltage;
        var heightFromGroundInches = heightFromStowInches + ClimberSetpoints.STOW;
        return heightFromGroundInches;
    }

    public double getHeightFromGroundInches() {
        switch (heightMode) {
            // case kRelative:
            // return getHeightFromGroundInchesUsingRelativeEncoder();
            case kAbsolute:
                return getHeightFromGroundInchesUsingAbsoluteEncoder();
            default:
                DriverStation.reportError("CLIMBER: Invalid mode for measuring height.", false);
                pid.reset();
                return setpointHeightFromGroundInches;
        }
    }

    //  (*º-º*)
    //   /   \
}
