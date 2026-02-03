package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConfig.DeployerConfig;


import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyWheels extends SubsystemBase {
    private final SparkMax motor;
    private final PIDController pid;

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
}
