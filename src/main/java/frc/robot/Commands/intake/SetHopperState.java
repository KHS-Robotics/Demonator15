package frc.robot.Commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Hopper.HopperState;

public class SetHopperState extends Command{

    private HopperState hopperState;
    private Intake intake;

    public SetHopperState(HopperState hopperState) {
        addRequirements(RobotContainer.kIntake);
        this.hopperState = hopperState;
        intake = RobotContainer.kIntake;
    }

    @Override
    public void initialize() {
        intake.setHopperState(hopperState);
    }
}
