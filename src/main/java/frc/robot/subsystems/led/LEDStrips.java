package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Degree;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LEDStrips extends SubsystemBase {
    private final BooleanSupplier IsAtSetpointAngle, IsAtSetpointDegrees;
    private final DoubleSupplier TurretDegrees;

    private int currentAnimationProgress = 0;
    private final Color RED = Color.fromHSV(0, 215, 255);
    private final Color BLUE = Color.fromHSV(115, 200, 255);

    private Color primaryColor;
    private Color secondaryColor;

    private final int ticksPerSecond = 20;

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

        Thread runLeds = new Thread(() -> {
      long lastTime = System.nanoTime();
      double delta = 0;
      while (!Thread.interrupted()) {
        double ns = 1000000000 / (double) ticksPerSecond;
        long now = System.nanoTime();
        delta += (now - lastTime) / ns;
        lastTime = now;
        if (delta >= 1) {
          runLEDS();
          delta--;
        }
      }
    });
    //shoutout jack for the crazy nanotime stuff i yoinked

        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();

        runLeds.start();
    }

    public void runLEDS(){
        runTurretLEDS();
    }

    public void runTurretLEDS(){
       if (RobotState.isEnabled()){
        runTurretLEDSEnabled();
       }
       else{
        runTurretLEDSDisabled();
       }
    }

    public void runTurretLEDSDisabled(){
        setAllToColor(Color.fromHSV(0, 0, 0), turretBufferView);
        if (currentAnimationProgress >= LEDConfig.kTurretLEDStripLength){
            currentAnimationProgress = 0;
        }
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
            Alliance coolAlliance = alliance.orElse(Alliance.Blue);
            if (coolAlliance == Alliance.Red){
                turretBufferView.setLED(currentAnimationProgress, RED);
            }
            else{
                turretBufferView.setLED(currentAnimationProgress, BLUE);
            }
        }
            currentAnimationProgress += animationProgress();
        }

    public void runTurretLEDSEnabled(){
        for (int i = 0; i < LEDConfig.kTurretLEDStripLength; i++){
            if (i <= traceTurretAngle()){
                turretBufferView.setLED(i, secondaryColor);
            }
            else{
                turretBufferView.setLED(i, primaryColor);
            }

        }
    }

    public int animationProgress(){
        int halfStripLength = (int) Math.round(LEDConfig.kTurretLEDStripLength / 2);
        int averageProgressPerTick = (int) Math.round(halfStripLength / LEDConfig.kTicksToHalfpointAnimationCycle);
        double weigh = 2 * (1 / Math.abs(halfStripLength - currentAnimationProgress));
        int tickProgress = (int) Math.round(averageProgressPerTick * weigh);
        return tickProgress;
    }


    public int getRGBValue(Color color, String channel){
        var value = 0.0;
        if (channel == "r"){
            value = color.red * 255;
        }
        else if (channel == "g"){
            value = color.green * 255;
        }
        else{
            value = color.blue * 255;
        }
        return (int) value;
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

    public void setAllToColor(Color color, AddressableLEDBufferView bufferView){
        for (int i = 0; i < bufferView.getLength(); i++){
            bufferView.setLED(i, color);
        }
    }

    public void initTurretSendable(SendableBuilder builder){
        super.initSendable(builder);
        builder.setSmartDashboardType("Turret LED");
    }

    public void initHopperSendable(SendableBuilder builder){
        super.initSendable(builder);
        builder.setSmartDashboardType("Hopper LED");
    }

}