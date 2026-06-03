

package frc.robot.Commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Hopper.HopperState;
import frc.robot.subsystems.intake.IntakeConfig.DeployerState;

public class setDeployerStateStow extends SequentialCommandGroup{

    private Intake intake = RobotContainer.kIntake;

    private Command moveDeployerWithWheelBoost = new SetDeployerState(DeployerState.kStow).alongWith(intake.intakeSlow());
    private Command moveHopperOut = new SetHopperState(HopperState.Deployed).withTimeout(0.6);
    private Command moveHopperIn = new SetHopperState(HopperState.Stowed).withTimeout(0.6);
    private Command setTurretOverride = RobotContainer.kTurret.setOverride(true, -90);

    private Command checkForBlockage = new ConditionalCommand(moveHopperOut, null, intake.hopperBlockingIntake());

    public setDeployerStateStow() {
        addRequirements(RobotContainer.kIntake, RobotContainer.kTurret);
        addCommands(
            setTurretOverride,
            checkForBlockage,
            moveDeployerWithWheelBoost,
            moveHopperIn
        );
    }
    
}