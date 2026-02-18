package frc.robot.subsystems.climber;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final Elevator innerElevator = new Elevator();


    public Climber() {
        SmartDashboard.putData(this);
    }

    public Command setClimberL1() {
        var cmd = innerElevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.L1);
        cmd.addRequirements(innerElevator);
        return cmd.withName("SetElevatorL1");
    }

    public Command setClimberL2() {
        var cmd = innerElevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.L2);
        cmd.addRequirements(innerElevator);
        return cmd.withName("SetElevatorL2");
    }

    public Command setClimberL3() {
        var cmd = innerElevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.L3);
        cmd.addRequirements(innerElevator);
        return cmd.withName("SetElevatorL3");
    }

    public Command setClimberStow() {
        var cmd = innerElevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.STOW);
        cmd.addRequirements(innerElevator);
        return cmd.withName("SetElevatorStow");
    }
    
    public Command deployOutsideHooks() {
        var cmd = innerElevator.deployOuterHooksCommand();
        cmd.addRequirements(innerElevator);
        return cmd.withName("deployOuterHooks");
    }

    public Command deployInsideHook() {
        var cmd = innerElevator.deployInnerHookCommand();
        cmd.addRequirements(innerElevator);
        return cmd.withName("deployInnerHooks");
    }

    public Command retractOutsideHooks() {
        var cmd = innerElevator.retractOuterHooksCommand();
        cmd.addRequirements(innerElevator);
        return cmd.withName("retractOuterHooks");
    }

    public Command retractInsideHooks() {
        var cmd = innerElevator.retractInnerHookCommand();
        cmd.addRequirements(innerElevator);
        return cmd.withName("retractInnerHooks");
    }

    public Command stowOutsideHooks() {
        var cmd = innerElevator.retractOuterHooksCommand();
        cmd.addRequirements(innerElevator);
        return cmd.withName("retractOuterHooks");
    }

    public Command stowAllHooks() {
        var cmd = innerElevator.StowAllHooksCommand();
        cmd.addRequirements(innerElevator);
        return cmd.withName("retractInnerHooks");
    }

    public Command goToL1(){
    var cmd = retractOutsideHooks().andThen(setClimberL1()).andThen(deployOutsideHooks()).andThen(setClimberL2()).andThen(retractInsideHooks());
     return cmd.withName("goToL1"); 
    }

    //we should have a prepare climb thing and an autoclimb, look into the elevator for how to do autoclimb
    
}
