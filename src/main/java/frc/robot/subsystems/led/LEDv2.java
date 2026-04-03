// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.RobotMap;

/** Add your docs here. */
public class LEDv2 {
    Thread t;
    AddressableLED strip;
    AddressableLEDBuffer buffer;
    int numberSections;
    int counter;
    int ticksPerSecond = 20;

    float speedFactor = 1f;
    float sections = 5f;
    float currentPosition = 0f;
    Color[] pixelArray;

    private final BooleanSupplier hoodCanHit, waistCanHit, aimingAtHub;

  public LEDv2 (BooleanSupplier hoodCanHit, BooleanSupplier waistCanHit, BooleanSupplier aimingAtHub) { 
    this.hoodCanHit = hoodCanHit;
    this.waistCanHit = waistCanHit;
    this.aimingAtHub = aimingAtHub;

    pixelArray = new Color[LEDConfig.LED_LENGTH];
    for (int i = 0; i < pixelArray.length; i++) {
      pixelArray[i] = new Color();
    }

    t = new Thread(() -> {
      long lastTime = System.nanoTime();
      double delta = 0;
      while (!Thread.interrupted()) {
        double ns = 1000000000 / (double) ticksPerSecond;
        long now = System.nanoTime();
        delta += (now - lastTime) / ns;
        lastTime = now;
        if (delta >= 1) {
        
          delta--;
        }
      }
    });
    
    strip = new AddressableLED(RobotMap.LED_PORT);
    strip.setLength(LEDConfig.LED_LENGTH);

    buffer = new AddressableLEDBuffer(LEDConfig.LED_LENGTH);
    strip.setData(buffer);
    strip.start();

    t.start();
 }

  public void setRGB(int index, int r, int g, int b) {
    buffer.setRGB(index, r, g, b);
  }

  public void runBlue() {
    LEDPattern blue = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlue, Color.kDarkBlue);
    blue.applyTo(buffer);
    strip.setData(buffer);
  }

  public void runRed() {
    LEDPattern red =LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kDarkRed);;
    red.applyTo(buffer);
    strip.setData(buffer);
  }

  public void runAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      LEDPattern.solid(Color.kWhite);
    } else {
      if (alliance.get() == Alliance.Blue) {
        runBlue();
      } else {
        runRed();
      }
    }
  }

  public void runDisabled(){
    runAllianceColor();
  }

  public void runEnabled(){
    if (aimingAtHub.getAsBoolean()){
      LEDPattern.solid(Color.kSalmon);
      if (waistCanHit.getAsBoolean() && hoodCanHit.getAsBoolean()){
        LEDPattern.solid(Color.kMintcream);
      }
      else {
       if (waistCanHit.getAsBoolean()){
         LEDPattern.solid(Color.kDarkMagenta);

        } else {
         LEDPattern.solid(Color.kLavender);
        }
      }
    }
  }

  public void getUpdate(){
    if(RobotState.isDisabled()){
      runDisabled();
    } else { 
      runEnabled();
    }
  } 

}
