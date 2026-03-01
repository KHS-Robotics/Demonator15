package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Deployer deployer = new Deployer();
    private final GrabbyWheels grabbyWheels = new GrabbyWheels();
    private final Hopper hopper = new Hopper();

    public Intake() {
        SmartDashboard.putData(this);
    }

    public Command agitate(){
        var cmd = this.extendDeployer().andThen(this.deployerToAgitate());
        cmd.addRequirements(this);
        return cmd.withName("AgitateIntake");
    }

    public Command deployerToAgitate(){
        var cmd = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE);
        cmd.addRequirements(this);
        return cmd.withName("SetIntakeStateToAgitate");
    }

    public Command deployDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand()
                .andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY);

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateDeploy");
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
 
    public Command intakeFuel() {
        var cmd = startEnd(grabbyWheels::intake, grabbyWheels::stop);
        cmd.addRequirements(grabbyWheels);
        return cmd.withName("IntakeFuel");
    }

    public Command outtakeFuel() {
        var cmd = startEnd(grabbyWheels::outake, grabbyWheels::stop);
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
