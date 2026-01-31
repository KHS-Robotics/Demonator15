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
   //indexerState = IndexerState.IDLE;
   //motor.stopMotor();
  //}

 //public void forward() {
   // indexerState = IndexerState.FORWARD;
   // motor.setVoltage(2.75);
  // }

 //public Command forwardCommand() {
   // var cmd = runOnce(this::start);
   // return cmd.withName("ForwardIndexer");
  //}

  //public void reverse() {
   // indexerState = IndexerState.REVERSE;
   // motor.setVoltage(-6);
  //}

  //public Command reverseCommand() {
    //var cmd = runOnce(this::reverse);
    //return cmd.withName("ReverseIndexer");
  //}

 //public enum IndexerState {
    //IDLE("Idle"),
    //FORWARD("Forward"),
    //REVERSE("Reverse"),

    //private final String state;

    //private Indexer(String s) {
      //state = s;
    //}

    //public String toString() {
    // return this.state;
    //}
  //}
// }
