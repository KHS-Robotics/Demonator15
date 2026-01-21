// package frc.robot.subsystems.intake;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.EncoderConfig;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Deployer extends SubsystemBase {
//     private final SparkMax motor;
//     private final AbsoluteEncoder encoder;
//     private final PIDController pid;

//     public Deployer(){
//
//super(Coraller.class.getSimpleName() + "/" + Angler.class.getSimpleName());

    // var encoderConfig = new AbsoluteEncoderConfig()
    //   .inverted(true);
    // var limitSwitchConfig = new LimitSwitchConfig()
    //   .forwardLimitSwitchEnabled(false)
    //   .reverseLimitSwitchEnabled(false);
    // var deployerConfig = new SparkMaxConfig()
    //   .idleMode(IdleMode.kBrake)
    //   .smartCurrentLimit(30)
    //   .inverted(false)
    //   .apply(limitSwitchConfig)
    //   .apply(encoderConfig);
    // motor = new SparkMax(RobotMap.INTAKE_DEPLOYER_ID, MotorType.kBrushless);
    // motor.configure(deployerConfig, SparkBase.ResetMode.kResetSafeParameters,
    //     SparkBase.PersistMode.kPersistParameters);

    // encoder = motor.getAbsoluteEncoder();
    // sensor = motor.getForwardLimitSwitch();

    // pid = new PIDController(DeployerConfig.kDeployerP, DeployerConfig.kDeployerI, DeployerConfig.kDeployerD);
    // pid.setIZone(7);

    // SmartDashboard.putData(getName(), this);
    // SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
//
// }
