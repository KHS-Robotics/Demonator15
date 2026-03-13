package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    public final Deployer deployer = new Deployer();
    private final GrabbyWheels grabbyWheels = new GrabbyWheels();
    private final Hopper hopper = new Hopper();


    public Intake() {
        SmartDashboard.putData(this);
    }

    public Command deployDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand()
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY);

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateDeploy");
    }

    public Command deployDeployerBangBang() {
        var hopperCurrentlyRetracted = 
                hopper.deployHopperCommand()
                .andThen(deployer.bangBangControlDeploy())
                .andThen(RobotContainer.kTurret.setOverride(false, 0));
        var hopperAlreadyDeployed = deployer.bangBangControlDeploy()
        .andThen(RobotContainer.kTurret.setOverride(false, 0));

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateDeployBang");
    }

    public Command agitateDeployerBangBang(){
        var cmd = deployer.bangBangControlAgitate().andThen(Commands.waitSeconds(2));
        return cmd;
    }

    public Command stowDeployerBangBang() {
        var hopperCurrentlyRetracted = RobotContainer.kTurret.setOverride(true, -90)
                .andThen(hopper.deployHopperCommand())
                .andThen(deployer.bangBangControlStow())
                .andThen(hopper.retractHopperCommand());
        var hopperAlreadyDeployed = RobotContainer.kTurret.setOverride(true, -90)
            .andThen(deployer.bangBangControlStow())
            .andThen(hopper.retractHopperCommand());

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateStowBang");
    }

    public Command stowDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand().andThen(
                deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW).andThen(hopper.retractHopperCommand()));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW)
                .andThen(hopper.retractHopperCommand());

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateStow");
    }

    public Command extendDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand()
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.EXTEND));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.EXTEND);

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateExtend");
    }
    /*
     * when calling this command in robotcontainer, make sure to add .repeatedly at the end 
     * to make sure it agitates continuously rather than just once. 
     */
    public Command agitateDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand()
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE_LOW)
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE_HIGH)));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE_LOW)
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE_HIGH));
        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("AgitateIntake");
    }

    public Command retractHopper() {
        return hopper.retractHopperCommand();
    }

    public Command deployHopper() {
        return hopper.deployHopperCommand();
    }
 
    public Command intakeFuel() {
        var cmd = runEnd(() -> {
            grabbyWheels.intake();
            deployer.keepDeployerDown();
        }, () -> {
            grabbyWheels.stop();
            deployer.stop();
        });
        cmd.addRequirements(grabbyWheels, deployer);
        return cmd.withName("IntakeFuel");
    }

    public Command outtakeFuel() {
        var cmd = runEnd(grabbyWheels::outtake, grabbyWheels::stop);
        cmd.addRequirements(grabbyWheels);
        return cmd.withName("OutakeFuel");
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        cmd.addRequirements(deployer, grabbyWheels);
        return cmd.withName("StopIntake");
    }

    public void stop() {
        deployer.stop();
        grabbyWheels.stop();
    }

    public Command stopGrabbyWheelsCommand() {
        var cmd = runOnce(this::stopGrabbyWheels);
        cmd.addRequirements(grabbyWheels);
        return cmd;
    }

    public void stopGrabbyWheels() {
        grabbyWheels.stop();
    }

    public Command fullyStow() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand()
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW))
                .andThen(hopper.retractHopperCommand());

        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW)
                .andThen(hopper.retractHopperCommand());

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("setStowDeployerAndHopper");
    }

    /** {@inheritDoc} */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.setSafeState(this::stop);
        builder.setActuator(true);
    }
}
