package frc.robot.Commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfig.DeployerState;

public class SetDeployerState extends Command {

    private DeployerState deployerState;
    private Intake intake;

    public SetDeployerState(DeployerState deployerState) {
        addRequirements(RobotContainer.kIntake);
        this.deployerState = deployerState;
        intake = RobotContainer.kIntake;
    }

    @Override
    public void initialize() {
        intake.setDeployerState(deployerState);
    }

    @Override
    public boolean isFinished() {
        return intake.deployerIsAtSetpoint();
    }
}
