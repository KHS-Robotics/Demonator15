// package frc.robot.subsystems.turret;

// public class Aimer {
//      private final SparkMax motor;
//      private final PIDController pid;
//      private final AbsoluteEncoder encoder;

//      public Aimer() {

//          var encoderConfig = new AbsoluteEncoderConfig()
//            .inverted(true);

//          var aimerConfig = new SparkMaxConfig()
//            .idleMode(IdleMode.kBrake)
//             .smartCurrentLimit(30)
//             .inverted(false)
//             .apply(limitSwitchConfig)
//             .apply(encoderConfig);

//          motor = new SparkMax(RobotMap.TURRET_AIMER_ID, MotorType.kBrushless);
//          motor.configure(aimerConfig, SparkBase.ResetMode.kResetSafeParameters,
//              SparkBase.PersistMode.kPersistParameters);

//          encoder = motor.getAbsoluteEncoder();

//          pid = new PIDController(AimerConfig.kAimerP, AimerConfig.kAimerI, AimerConfig.kAimerD);
//      }
// }

// one yaw adjustment motor
// one pitch adjustment motor