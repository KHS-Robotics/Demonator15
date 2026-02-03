package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig.WaistConfig;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Waist {
    private final SparkMax motor;
    private final PIDController pid;
    private final AbsoluteEncoder encoder;

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
}
// yaw adjustment
// o–(•o•)–o
//   /   \
//   o   o yay!!!
