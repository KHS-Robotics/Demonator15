// package frc.robot.subsystems.turret;

// public class Kicker {

//     private final SparkMax motor;

//     public Kicker() {
    //     super(Kicker.class.getSimpleName() + "/" + Kicker.class.getSimpleName());
        
    //     var motorConfig = new SparkMaxConfig()
    //         .idleMode(IdleMode.kBrake)
    //         .smartCurrentLimit(30)
    //         .inverted(true);
    //     motor = new SparkMax(RobotMap.TURRET_KICKER_MOTOR_ID, MotorType.kBrushless);
    //     motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
    //         SparkBase.PersistMode.kPersistParameters); 



    //     SmartDashboard.putData(getName(), this);
    //     }

    //     public void stop() {
    //         motor.stopMotor();
    //     }

    //     public Command stopCommand() {
    //         var cmd = runOnce(this::stop);
    //         return cmd.withName("StopIntake");
    //     }

    //     public void spit() {
    //         motor.setVoltage(2.75);
    //     }

    //     public Command spitCommand() {
    //         var cmd = runOnce(this::start);
    //         return cmd.withName("StartIntake");
    //     }

//     }
// }


//indexer -> tower motor  <- put the belt into indexer

//pre-shooter motor
