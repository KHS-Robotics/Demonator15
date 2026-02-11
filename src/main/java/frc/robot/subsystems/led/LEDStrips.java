package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Degree;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class LEDStrips extends SubsystemBase {
    private final BooleanSupplier IsAtSetpointAngle, IsAtSetpointDegrees;
    private final DoubleSupplier TurretDegrees;

    AddressableLEDBufferView turretBufferView;
    AddressableLEDBufferView hopperBufferView;
    AddressableLED addressableLED;
    AddressableLEDBuffer addressableLEDBuffer;
    public LEDStrips(BooleanSupplier IsAtSetpointAngle, BooleanSupplier IsAtSetpointDegrees, DoubleSupplier TurretDegrees){
        this.IsAtSetpointAngle = IsAtSetpointAngle;
        this.IsAtSetpointDegrees = IsAtSetpointDegrees;
        this.TurretDegrees = TurretDegrees;

        addressableLED = new AddressableLED(RobotMap.LED_PORT);
        addressableLED.setLength(LEDConfig.kTotalLEDStripLength);

        addressableLEDBuffer = new AddressableLEDBuffer(LEDConfig.kTotalLEDStripLength);

        turretBufferView = addressableLEDBuffer.createView(0, LEDConfig.kTurretLEDStripLength);
        hopperBufferView = addressableLEDBuffer.createView(LEDConfig.kTurretLEDStripLength + 1, LEDConfig.kTotalLEDStripLength);

        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    public void setTheme(Color primaryColor, Color secondaryColor){
        Color colorPrimary = primaryColor;
        Color colorSecondary = secondaryColor;
    }

    public void setHSV(AddressableLEDBufferView bufferView, int index, int h, int s, int v){
        bufferView.setHSV(index, h, s, v);
    }

    public void setRGB(AddressableLEDBufferView bufferView, int index, int r, int g, int b){
        bufferView.setRGB(index, r, g, b);
    }

    public void ledOff(AddressableLEDBufferView bufferView, int index){
        setRGB(bufferView, index, 0, 0, 0);
    }
    /**
     * 
     * @param index
     * @return degrees  of the given index led
     */
    public double getTurretLEDDegrees(int index){
        return 360 / LEDConfig.kTurretLEDStripLength * index;
    }

    public boolean isTurretAtSetpoint(boolean isHoodAtSetpoint, boolean isWaistAtSetpoint){
        return isHoodAtSetpoint && isWaistAtSetpoint;
    }

    /**
     * Tracks which leds are at or below the turret's angle.
     * @return The largest index of the turret LED strip whose angle does not exceed the turret's
     */
    public int traceTurretAngle(){
        for (int i = 0; i < LEDConfig.kTurretLEDStripLength; i++){
            if (getTurretLEDDegrees(i) > TurretDegrees.getAsDouble()){
                return i - 1;
            }
        }
        return 0;
    }


}