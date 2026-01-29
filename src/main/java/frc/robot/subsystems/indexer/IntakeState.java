// package frc.robot.subsystems.indexer;

// class Indexer extends SubsystemBase{
//      private final SparkMax motor;
//
//      public Indexer() {
//          super(Indexer.class.getSimpleName() + "/" + Indexer.class.getSimpleName());
    //
    //      var hopperConfig = new SparkMaxConfig()
    //          .idleMode(IdleMode.kBrake)
    //          .smartCurrentLimit(30)
    //          .inverted(true);
    //      motor = new SparkMax(RobotMap.HOPPER_MOTOR_ID, MotorType.kBrushless);
    //      motor.configure(hopperConfig, SparkBase.ResetMode.kResetSafeParameters,
    //          SparkBase.PersistMode.kPersistParameters);
//
//
//      }
// 
// public Command stopCommand() {
   // return runOnce(this::stop)
   // .withName("StopIndexer");
  // }

  // public void stop() {
   //intakeState = IntakeState.IDLE;
   //motor.stopMotor();
  //}

//public void start() {
   // intakeState = IntakeState.INTAKING;
   // motor.setVoltage(2.75);
  // }

//public Command intakeCommand() {
   // var cmd = runOnce(this::start);
   // return cmd.withName("StartIndexer");
  //}

//public void reverse() {
   //intakeState = IntakeState.OUTAKING;
   //motor.setVoltage(-12);
  // }

  //public Command outtakeCommand() {
    //var cmd = runOnce(this::reverse);
    //return cmd.withName("ReverseIndexer");
  //}

 //public enum IntakeState {
    //IDLE("Idle"),
    //INTAKING("Intaking"),
    //OUTAKING("Outaking");

    //private final String state;

    //private IntakeState(String s) {
      //state = s;
    //}

    //public String toString() {
    // return this.state;
    //}
  //}
// }
