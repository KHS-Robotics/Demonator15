package frc.robot.subsystems.indexer;
import com.revrobotics.PersistMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.revrobotics.spark.SparkLowLevel.MotorType;


class Indexer extends SubsystemBase {
  private IndexerState indexerState = IndexerState.IDLE;
  private final SparkMax motor;

  public Indexer() {
    super(Indexer.class.getSimpleName() + "/" + Indexer.class.getSimpleName());

    var hopperConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true);
    motor = new SparkMax(RobotMap.HOPPER_MOTOR_ID, MotorType.kBrushless);
    motor.configure(hopperConfig, ResetMode.kResetSafeParameters,
     PersistMode.kPersistParameters);

  }

  public Command stopCommand() {
    return runOnce(this::stop)
        .withName("StopIndexer");
  }

  public void stop() {
    indexerState = IndexerState.IDLE;
    motor.stopMotor();
  }

  public void forward() {
    indexerState = IndexerState.FORWARD;
    motor.setVoltage(2.75);
  }

  public Command forwardCommand() {
    var cmd = runOnce(this::forward);
    return cmd.withName("ForwardIndexer");
  }

  public void reverse() {
    indexerState = IndexerState.REVERSE;
    motor.setVoltage(-6);
  }

  public Command reverseCommand() {
    var cmd = runOnce(this::reverse);
    return cmd.withName("ReverseIndexer");
  }

  public enum IndexerState {
    IDLE("Idle"),
    FORWARD("Forward"),
    REVERSE("Reverse");
  

   private final String state;

  private IndexerState(String s) {
     state = s;
    }

    public String toString() {
      return this.state;
    }
  }
}
