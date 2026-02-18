package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Degree;

import java.awt.List;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LEDStrips extends SubsystemBase {
    private final BooleanSupplier IsAtSetpointAngle, IsAtSetpointDegrees;
    private final DoubleSupplier TurretDegrees;

    int animationSelectedLED;

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

    public void runTurretLEDS(){
        if (RobotState.isDisabled()){
            runTurretLEDSDisabled();
        }

    }

    public void setToAllianceColor(){
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == Alliance.Red){
            setTheme(Color.fromHSV(0, 215, 255), Color.fromHSV(25, 215, 255), turretBufferView);
            setTheme(Color.fromHSV(0, 215, 255), Color.fromHSV(25, 215, 255), hopperBufferView);
        }
        else{
            setTheme(Color.fromHSV(115, 200, 255), Color.fromHSV(95, 230, 255), turretBufferView);
            setTheme(Color.fromHSV(115, 200, 255), Color.fromHSV(95, 230, 255), hopperBufferView);
        }
        for (int i = 0; i < LEDConfig.kTurretLEDStripLength; i++){
            turretBufferView.setRGB(i, rgbConversion(turretColorPrimary.red), rgbConversion(turretColorPrimary.green), rgbConversion(turretColorPrimary.blue));
        }
        for (int i = LEDConfig.kTurretLEDStripLength; i < LEDConfig.kHopperLEDStripLength; i++){
            hopperBufferView.setRGB(i, rgbConversion(hopperColorPrimary.red), rgbConversion(hopperColorPrimary.green), rgbConversion(hopperColorPrimary.blue));
        }
    }


    public void disabledAnimation(){
        turretBufferView.setRGB(animationSelectedLED, rgbConversion(turretColorSecondary.red), rgbConversion(turretColorSecondary.green), rgbConversion(turretColorSecondary.blue));
        for (int i = 0; i < LEDConfig.kTicksPerAnimationCycle; i++){
                turretBufferView.setRGB(selectedRGB, rgbConversion(turretColorSecondary.red), rgbConversion(turretColorSecondary.green), rgbConversion(turretColorSecondary.blue));
            }
        }

    public int animationProgress(int tick){
        double doubleStripLength = LEDConfig.kTurretLEDStripLength;
        long tickProgress = Math.round(doubleStripLength / LEDConfig.kTicksPerAnimationCycle * MathUtil.interpolate(0.0, doubleStripLength, Math.abs(1 / (tick - doubleStripLength / 2))));
        return (int) tickProgress;
    }

    public int rgbConversion(double percent){
        return (int) Math.round(255 * percent);
    }

    public Boolean secondaryColorEnabled = true;
    Color turretColorPrimary = Color.fromHSV(234, 94, 100);
    Color turretColorSecondary = Color.fromHSV(191, 91, 100);
    Color hopperColorPrimary = Color.fromHSV(234, 94, 100);
    Color hopperColorSecondary = Color.fromHSV(191, 91, 100);

    public void setTheme(Color primaryColor, Color secondaryColor, AddressableLEDBufferView bufferView){
        if (bufferView == turretBufferView){
            turretColorPrimary = primaryColor;
            if (secondaryColorEnabled){
                turretColorSecondary = secondaryColor;
            }
            else{
                turretColorSecondary = Color.fromHSV(0, 0, 0);
            }
        }
        else if(bufferView == hopperBufferView){
            hopperColorPrimary = primaryColor;
            if (secondaryColorEnabled){
                hopperColorSecondary = secondaryColor;
            }
            else{
                hopperColorSecondary = Color.fromHSV(0, 0, 0);
            }
        }
    }

    public Double getRGBValue(Color color, String channel){
        var value = 0.0;
        if (channel == "r"){
            value = color.red;
        }
        else if (channel == "g"){
            value = color.green;
        }
        else{
            value = color.blue;
        }
        return value;
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
    /** Gets the
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
     * Tracks which leds are at or below the turret's angle. Assumes the first LED in the strip is at degree zero.
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

    public void initTurretSendable(SendableBuilder builder){
        super.initSendable(builder);
        builder.setSmartDashboardType("Turret LED");
        builder.addDoubleProperty("Primary Color R", () -> this.getRGBValue(turretColorPrimary, "r"), null);
        builder.addDoubleProperty("Primary Color G", () -> this.getRGBValue(turretColorPrimary, "g"), null);
        builder.addDoubleProperty("Primary Color B", () -> this.getRGBValue(turretColorPrimary, "b"), null);
        builder.addDoubleProperty("Secondary Color R", () -> this.getRGBValue(turretColorSecondary, "r"), null);
        builder.addDoubleProperty("Secondary Color G", () -> this.getRGBValue(turretColorSecondary, "g"), null);
        builder.addDoubleProperty("Secondary Color B", () -> this.getRGBValue(turretColorSecondary, "b"), null);
    }

    public void initHopperSendable(SendableBuilder builder){
        super.initSendable(builder);
        builder.setSmartDashboardType("Hopper LED");
        builder.addDoubleProperty("PrimaryColor R", () -> this.getRGBValue(hopperColorPrimary, "r"), null);
        builder.addDoubleProperty("PrimaryColor G", () -> this.getRGBValue(hopperColorPrimary, "g"), null);
        builder.addDoubleProperty("PrimaryColor B", () -> this.getRGBValue(hopperColorPrimary, "b"), null);
        builder.addDoubleProperty("SecondaryColor R", () -> this.getRGBValue(hopperColorSecondary, "r"), null);
        builder.addDoubleProperty("SecondaryColor G", () -> this.getRGBValue(hopperColorSecondary, "g"), null);
        builder.addDoubleProperty("SecondaryColor B", () -> this.getRGBValue(hopperColorSecondary, "b"), null);
    }

}