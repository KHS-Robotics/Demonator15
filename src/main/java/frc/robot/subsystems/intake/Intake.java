package frc.robot.subsystems.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private final Deployer deployer = new Deployer();
    private final GrabbyWheels grabbyWheels = new GrabbyWheels();
    private final Servo rightAcuator, leftAcuator;

    public Intake() {
        SmartDashboard.putData(this);

        rightAcuator = new Servo(RobotMap.RIGHT_SERVO);
        leftAcuator = new Servo(RobotMap.LEFT_SERVO);
    }

    public Command deployDeployer() {
        var cmd = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.DEPLOY);
        cmd.addRequirements(deployer);
        return cmd.withName("DeployIntake");
    }

    public Command stowDeployer() {
        var cmd = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.STOW);
        cmd.addRequirements(deployer);
        return cmd.withName("StowIntake");
    }

    public Command agitateDeployer() {
        var cmd = deployer.setAngleCommand(IntakeConfig.DeployerSetpoints.AGITATE);
        cmd.addRequirements(deployer);
        return cmd.withName("AgitateIntake");
    }

    public void retractLinearAcuators() {
        rightAcuator.set(0);
        leftAcuator.set(0);
    }

    public void deployLinearAcuators(){
        rightAcuator.set(0);
        leftAcuator.set(0);
    }

    public Command intakefuel() {
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

    /** {@inheritDoc} */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.setSafeState(this::stop);
        builder.setActuator(true);
    }
}
