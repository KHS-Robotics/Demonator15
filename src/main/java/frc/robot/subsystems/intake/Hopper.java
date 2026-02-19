package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConfig.HopperConfig;

/**
 * Holds all the fuel yo
 */
public class Hopper extends SubsystemBase {
    public enum HopperState {
        Retracted, Deployed;
    }

    private HopperState state = HopperState.Retracted;
    private final Servo rightAcuator, leftAcuator;

    public Hopper() {
        SmartDashboard.putData(this);

        rightAcuator = new Servo(RobotMap.RIGHT_SERVO);
        leftAcuator = new Servo(RobotMap.LEFT_SERVO);
    }

    /** {@inheritDoc}} */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());
        builder.addBooleanProperty("IntakeBlocked", isBlockingIntake(), null);
    }

    /**
     * Gets the current state of the hopper position.
     * 
     * @return the hopper state
     * @see HopperState
     */
    public Supplier<HopperState> state() {
        return () -> state;
    }

    /**
     * Gets if the hopper is currently retracted
     * 
     * @return true if retracted, false otherwise
     * @see #state()
     */
    public BooleanSupplier isBlockingIntake() {
        return () -> this.state().get() == HopperState.Retracted;
    }

    public Command retract() {
        var setHopperState = runOnce(() -> state = HopperState.Retracted);
        var retractCmd = runOnce(() -> {
            rightAcuator.set(HopperConfig.kRetractServoPosition.magnitude());
            leftAcuator.set(HopperConfig.kRetractServoPosition.magnitude());
        });
        var cmd = setHopperState
                .andThen(retractCmd)
                .andThen(Commands.waitSeconds(HopperConfig.kSecondsBetweenStates.magnitude()));
        return cmd;
    }

    public Command deploy() {
        var setHopperState = runOnce(() -> state = HopperState.Deployed);
        var retractCmd = runOnce(() -> {
            rightAcuator.set(HopperConfig.kDeployServoPosition.magnitude());
            leftAcuator.set(HopperConfig.kDeployServoPosition.magnitude());
        });
        var cmd = setHopperState
                .andThen(retractCmd)
                .andThen(Commands.waitSeconds(HopperConfig.kSecondsBetweenStates.magnitude()));
        return cmd;
    }
}
