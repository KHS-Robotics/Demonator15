// package frc.robot.subsystems.turret;

// public class Waist {
//      private final SparkMax motor;
//      private final PIDController pid;
//      private final AbsoluteEncoder encoder;

//      public Waist() {

//          var encoderConfig = new AbsoluteEncoderConfig()
//            .inverted(true);

//          var waistConfig = new SparkMaxConfig()
//             .idleMode(IdleMode.kBrake)
//             .smartCurrentLimit(30)
//             .inverted(false)
//             .apply(encoderConfig);

//          motor = new SparkMax(RobotMap.TURRET_AIMER_WAIST_ID, MotorType.kBrushless);
//          motor.configure(waistConfig, SparkBase.ResetMode.kResetSafeParameters,
//              SparkBase.PersistMode.kPersistParameters);

//          encoder = motor.getAbsoluteEncoder();

//          pid = new PIDController(WaistConfig.kWaistP, WaistConfig.kWaistI, WaistConfig.kWaistD);
//      }
// }
//yaw adjustment
