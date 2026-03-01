package frc.robot.subsystems.indexer;
import com.revrobotics.PersistMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Indexer extends SubsystemBase {
  private IndexerState indexerState = IndexerState.IDLE;
  private final SparkMax motor;

  public Indexer() {
    super(Indexer.class.getSimpleName() + "/" + Indexer.class.getSimpleName());

    var indexerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false);
    motor = new SparkMax(RobotMap.HOPPER_MOTOR_ID, MotorType.kBrushless);
    motor.configure(indexerConfig, ResetMode.kResetSafeParameters,
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
    motor.setVoltage(12);
  }

  public Command forwardCommand() {
    var cmd = startEnd(this::forward, this::stop);
    return cmd.withName("ForwardIndexer");
  }

  public void reverse() {
    indexerState = IndexerState.REVERSE;
    motor.setVoltage(-6);
  }

  public Command reverseCommand() {
    var cmd = startEnd(this::reverse, this::stop);
    return cmd.withName("ReverseIndexer");
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.addStringProperty("IdexerState", () -> indexerState.toString(), null);
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
