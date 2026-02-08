package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretLEDStrip extends SubsystemBase {
    private final BooleanSupplier IsAtSetpoint;
    private final DoubleSupplier Degrees;
    public TurretLEDStrip(BooleanSupplier IsAtSetpoint, DoubleSupplier Degrees){
        this.IsAtSetpoint = IsAtSetpoint;
        this.Degrees = Degrees;
    }
}
