// package frc.robot.subsystems.turret;

// public class Spitter {

  // private final SparkMax leader, follower;

  // public Spitter() {
  //   super(Spitter.class.getSimpleName() + "/" + Spitter.class.getSimpleName());
    
  //   var leaderConfig = new SparkMaxConfig()
  //     .idleMode(IdleMode.kBrake)
  //     .smartCurrentLimit(30)
  //     .inverted(true);
  //   leader = new SparkMax(RobotMap.TURRET_SPITTER_LEADER_MOTOR_ID, MotorType.kBrushless);
  //   leader.configure(leaderConfig, SparkBase.ResetMode.kResetSafeParameters,
  //       SparkBase.PersistMode.kPersistParameters); 

  //  var followerConfig = new SparkMaxConfig()
  //     .ildeMode(IdleMode.kBrake)
  //     .smartCurrentLimit(30)
  //     .follow(RobotMap.TURRET_SPITTER_LEADER_ID, true)
  //   follower = new SparkMax(RobotMap.TURRET_SPITTER_FOLLOWER_ID, MotorType.kBrushless);
  //   follower.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
  //       SparkBase.PersistMode.kPersistParameters);


  //   SmartDashboard.putData(getName(), this);
  // }

  // public void stop() {
  //   motor.stopMotor();
  // }

  // public Command stopCommand() {
  //   var cmd = runOnce(this::stop);
  //   return cmd.withName("StopIntake");
  // }

  // public void spit() {
  //   motor.setVoltage(2.75);
  // }

  // public Command spitCommand() {
  //   var cmd = runOnce(this::start);
  //   return cmd.withName("StartIntake");
  // }

//two shooter motors
//