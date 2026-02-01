// package frc.robot.subsystems.turret;

// public class Hood {
//      private final SparkMax motor;
//      private final PIDController pid;
//      private final AbsoluteEncoder encoder;

//      public Hood() {

//          var encoderConfig = new AbsoluteEncoderConfig()
//            .inverted(true);

//          var hoodConfig = new SparkMaxConfig()
//             .idleMode(IdleMode.kBrake)
//             .smartCurrentLimit(30)
//             .inverted(false)
//             .apply(encoderConfig);

//          motor = new SparkMax(RobotMap.TURRET_AIMER_HOOD_ID, MotorType.kBrushless);
//          motor.configure(hoodConfig, SparkBase.ResetMode.kResetSafeParameters,
//              SparkBase.PersistMode.kPersistParameters);

//          encoder = motor.getAbsoluteEncoder();

//          pid = new PIDController(HoodConfig.kHoodP, HoodConfig.kHoodI, HoodConfig.kHoodD);
//      }
// }

//pitch adjustment 