package frc.robot.subsystems.turret;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig.HoodConfig;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hood {
     private final SparkMax motor;
     private final PIDController pid;
     private final AbsoluteEncoder encoder;

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

         pid = new PIDController(HoodConfig.kAimerHoodP, HoodConfig.kAimerHoodI, HoodConfig.kAimerHoodD);
     }
}

//pitch adjustment 