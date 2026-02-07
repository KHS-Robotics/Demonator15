package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private final Hood hood = new Hood();
    private final Waist waist = new Waist();
    private final Kicker kicker = new Kicker();
    private final Spitter spitter = new Spitter();

    public Turret() {
        //SmartDashboard.putData(this);
    }

    public Command shootMomentarily() {
        var cmd = startEnd(spitter::start, spitter::stop);
        cmd.addRequirements(spitter);
        return cmd;
    }

    public Command reload() {
        var cmd = startEnd(kicker::start, kicker::stop);
        cmd.addRequirements(kicker);
        return cmd;
    }

    public Command shootContinuously() {
        var cmd = Commands.parallel(
                shootMomentarily(),
                reload()
            );
        cmd.addRequirements(this);
        return cmd;
    }
    //for getting the position and rotation (which we will probably want to do in hood and waist), use our humble kNavx
    //kNavx.getVelocityX and VelocityY
    //kNavx.getRotation2d
    //find a way to get global position
}
