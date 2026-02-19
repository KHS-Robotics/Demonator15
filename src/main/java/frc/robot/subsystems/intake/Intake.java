package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Deployer deployer = new Deployer();
    private final Hopper hopper = new Hopper();
    private final GrabbyWheels grabbyWheels = new GrabbyWheels();

    public Intake() {
        SmartDashboard.putData(this);
    }

    public Command fullyStow() {
        // conditional collision logic for hopper+intake
        var onHopperCurrentlyRetracted = hopper.deploy().andThen(deployer.stow()).andThen(hopper.retract());
        var onHopperAlreadyDeployed = deployer.stow().andThen(hopper.retract());
        var cmd = new ConditionalCommand(onHopperCurrentlyRetracted, onHopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("StowDeployerAndHopper");
    }

    public Command deployDeployer() {
        // conditional collision logic for hopper+intake
        var onHopperCurrentlyRetracted = hopper.deploy().andThen(deployer.deploy());
        var onHopperAlreadyDeployed = deployer.deploy();
        var cmd = new ConditionalCommand(onHopperCurrentlyRetracted, onHopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("DeployDeployer");
    }

    public Command stowDeployer() {
        // conditional collision logic for hopper+intake
        var onHopperRetracted = hopper.deploy().andThen(deployer.stow());
        var onHopperAlreadyDeployed = deployer.stow();
        var cmd = new ConditionalCommand(onHopperRetracted, onHopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("StowDeployer");
    }

    public Command agitateDeployer() {
        // conditional collision logic for hopper+intake
        var onHopperRetracted = hopper.deploy().andThen(deployer.agitate());
        var onHopperAlreadyDeployed = deployer.agitate();
        var cmd = new ConditionalCommand(onHopperRetracted, onHopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("AgitateDeployer");
    }

    public Command intakeFuel() {
        var cmd = startEnd(grabbyWheels::intake, grabbyWheels::stop);
        return cmd.withName("IntakeFuel");
    }

    public Command outtakeFuel() {
        var cmd = startEnd(grabbyWheels::outake, grabbyWheels::stop);
        return cmd.withName("OutakeFuel");
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
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

    /** {@inheritDoc} */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.setSafeState(this::stop);
        builder.setActuator(true);
    }
}
