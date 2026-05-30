

package frc.robot.Commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfig.DeployerState;

public class setDeployerStateStow extends SequentialCommandGroup{

    private Intake intake = RobotContainer.kIntake;

    private Command moveDeployer = new SetDeployerState(DeployerState.kStow);
    private Command moveHopperOut = 

    private Command checkForBlockage = new ConditionalCommand(null, null, intake.hopperBlockingIntake());

    
}