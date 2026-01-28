// package frc.robot.subsystems.intake;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.EncoderConfig;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Deployer extends SubsystemBase {
//     private final SparkMax motorLead, motorFollower;
//   ^ There are two seperate motors for the intake pivot,
//might want to change right/left names to avoid confusion
//     private final RelativeEncoder relativeEncoder;
//     private final PIDController pid;
//     private final SparkLimitSwitch bottomLimitSwitch;

//     public Deployer(){
//
//super(Coraller.class.getSimpleName() + "/" + Deployer.class.getSimpleName());

//var relativeEncoderConfig = new EncoderConfig()
//       .positionConversionFactor(DeployerConfig.kDeployerEncoderPositionConversionFactor)
//       .velocityConversionFactor(DeployerConfig.kDeployerEncoderVelocityConversionFactor);
//

    // var limitSwitchConfig = new LimitSwitchConfig()
    //   .forwardLimitSwitchEnabled(false)
    //   .reverseLimitSwitchEnabled(false);

    // var deployerLeaderConfig = new SparkMaxConfig()
    //   .idleMode(IdleMode.kBrake)
    //   .smartCurrentLimit(30)
    //   .inverted(false)
    //   .apply(limitSwitchConfig)
    //   .apply(relativeEncoderConfig);
    // leader = new SparkMax(RobotMap.INTAKE_DEPLOYER_LEADER_ID, MotorType.kBrushless);
    // leader.configure(deployerLeaderConfig, SparkBase.ResetMode.kResetSafeParameters,
    //     SparkBase.PersistMode.kPersistParameters);

    // var followerConfig = new SparkMaxConfig()
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(30)
    //     .follow(RobotMap.INTAKE_DEPLOYER_LEADER_ID, true)
    //     .apply(relativeEncoderConfig);
    // follower = new SparkMax(RobotMap.INTAKE_DEPLOYER_FOLLOWER_ID, MotorType.kBrushless);
    // follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
    //      SparkBase.PersistMode.kPersistParameters);

    // encoder = leader.getEncoder();
    // sensor = motor.getForwardLimitSwitch();

    // pid = new PIDController(DeployerConfig.kDeployerP, DeployerConfig.kDeployerI, DeployerConfig.kDeployerD);
    // pid.setIZone(7);

    // SmartDashboard.putData(getName(), this);
    // SmartDashboard.putData(getName() + "/" + PIDController.class.getSimpleName(), pid);
//
// }
