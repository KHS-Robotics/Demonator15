package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private final Deployer deployer = new Deployer();
    private final GrabbyWheels grabbyWheels = new GrabbyWheels();
    private final Hopper hopper = new Hopper();

    public Intake() {
        SmartDashboard.putData(this);
    }

    public Command deployDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand().andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY);

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateDeploy");
    }

    public Command stowDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand().andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW);

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateStow");
    }

    public Command agitateDeployer() {
        var hopperCurrentlyRetracted = hopper.deployHopperCommand().andThen(deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE));
        var hopperAlreadyDeployed = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE);

        var cmd = new ConditionalCommand(hopperCurrentlyRetracted, hopperAlreadyDeployed, hopper.isBlockingIntake());
        return cmd.withName("SetDeployerStateAgitate");
    }

    public Command intakefuel() {
        var cmd = startEnd(grabbyWheels::intake, grabbyWheels::stop);
        cmd.addRequirements(grabbyWheels);
        return cmd.withName("IntakeFuel");
    }

    public Command outakefuel() {
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
