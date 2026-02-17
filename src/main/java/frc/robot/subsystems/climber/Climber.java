package frc.robot.subsystems.climber;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final Elevator elevator = new Elevator();

    public Climber() {
        SmartDashboard.putData(this);
    }

    public Command setClimberL1() {
        var cmd = elevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.L1);
        cmd.addRequirements(elevator);
        return cmd.withName("SetElevatorL1");
    }

    public Command setClimberL2() {
        var cmd = elevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.L2);
        cmd.addRequirements(elevator);
        return cmd.withName("SetElevatorL2");
    }

    public Command setClimberL3() {
        var cmd = elevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.L3);
        cmd.addRequirements(elevator);
        return cmd.withName("SetElevatorL3");
    }

    public Command setClimberStow() {
        var cmd = elevator.setHeightCommand(ClimberConfig.ElevatorSetpoints.STOW);
        cmd.addRequirements(elevator);
        return cmd.withName("SetElevatorStow");
    }
    
    public Command deployOutsideHooks() {
        var cmd = elevator.deployOuterHooksCommand();
        cmd.addRequirements(elevator);
        return cmd.withName("deployOuterHooks");
    }

    public Command deployInsideHook() {
        var cmd = elevator.deployInnerHookCommand();
        cmd.addRequirements(elevator);
        return cmd.withName("deployInnerHooks");
    }

    public Command retractOutsideHooks() {
        var cmd = elevator.retractOuterHooksCommand();
        cmd.addRequirements(elevator);
        return cmd.withName("retractOuterHooks");
    }

    public Command retractInsideHooks() {
        var cmd = elevator.retractInnerHookCommand();
        cmd.addRequirements(elevator);
        return cmd.withName("retractInnerHooks");
    }

    public Command stowOutsideHooks() {
        var cmd = elevator.retractOuterHooksCommand();
        cmd.addRequirements(elevator);
        return cmd.withName("retractOuterHooks");
    }

    public Command stowAllHooks() {
        var cmd = elevator.StowAllHooksCommand();
        cmd.addRequirements(elevator);
        return cmd.withName("retractInnerHooks");
    }

    //public Command goToL1(){
    //var cmd = retractOutsideHooks().andThen(setClimberL1()).andThen(deployOutsideHooks());
     //return cmd.withName("goToL1");
    //}

    //we should have a prepare climb thing and an autoclimb, look into the elevator for how to do autoclimb
    
}
