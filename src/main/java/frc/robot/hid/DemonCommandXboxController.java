// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}.
 * 
 * @see edu.wpi.first.wpilibj2.command.button.CommandXboxController
 */
public class DemonCommandXboxController extends CommandXboxController {

  /** Ranges from [0, 1] where 0 is full linear and 1 is full cubic. */
  public static final double kJoystickSensitivity = 0.5;

  /** Deadband since joysticks vary in how well they snap back to zero. */
  public static final double kJoystickDeadband = 0.035;

  public DemonCommandXboxController(int port) {
    super(port);
  }

  /**
   * Translational speeds for swerve teleop from the left stick (robot-forward X, robot-left Y).
   *
   * @param maxTranslationalMetersPerSecond scale at full deflection
   */
  public DriveTranslation getDriveTranslation(
      double deadband, double sensitivity, double maxTranslationalMetersPerSecond) {
    return driveTranslationFromAxes(
        getLeftY(),
        getLeftX(),
        deadband,
        sensitivity,
        maxTranslationalMetersPerSecond);
  }

  /** @see #getDriveTranslation(double, double, double) using {@link #kJoystickDeadband} and {@link #kJoystickSensitivity} */
  public DriveTranslation getDriveTranslation(double maxTranslationalMetersPerSecond) {
    return getDriveTranslation(kJoystickDeadband, kJoystickSensitivity, maxTranslationalMetersPerSecond);
  }

  /** Angular rate for swerve teleop from the right stick X axis. */
  public double getDriveRotation(double deadband, double sensitivity, double maxAngularRadiansPerSecond) {
    return rotationFromAxis(getRightX(), deadband, sensitivity, maxAngularRadiansPerSecond);
  }

  /**
   * Field-relative translation sign for red alliance ({@code -1}) vs robot-relative or blue ({@code 1}).
   */
  public static int fieldRelativeAllianceSign(boolean fieldRelative, Alliance alliance) {
    return fieldRelative && alliance == Alliance.Red ? -1 : 1;
  }

  /** @see #fieldRelativeAllianceSign(boolean, Alliance) using {@link DriverStation#getAlliance()} */
  public static int fieldRelativeAllianceSign(BooleanSupplier fieldRelative) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return fieldRelativeAllianceSign(fieldRelative.getAsBoolean(), alliance);
  }

  /**
   * Pure left-stick mapping for unit tests (Xbox: forward is {@code leftY = -1}).
   */
  static DriveTranslation driveTranslationFromAxes(
      double leftY,
      double leftX,
      double deadband,
      double sensitivity,
      double maxTranslationalMetersPerSecond) {
    double xSpeed = HIDUtils.scaledStickAxis(-leftY, deadband, sensitivity, maxTranslationalMetersPerSecond);
    double ySpeed = HIDUtils.scaledStickAxis(-leftX, deadband, sensitivity, maxTranslationalMetersPerSecond);
    return new DriveTranslation(xSpeed, ySpeed);
  }

  /** Pure right-stick X mapping for unit tests. */
  static double rotationFromAxis(
      double rightX,
      double deadband,
      double sensitivity,
      double maxAngularRadiansPerSecond) {
    return HIDUtils.scaledStickAxis(-rightX, deadband, sensitivity, maxAngularRadiansPerSecond);
  }

  /** Robot-frame translational speeds after deadband and scaling (m/s). */
  public record DriveTranslation(double xMetersPerSecond, double yMetersPerSecond) {}

  public Trigger resetRobotHeading() {
    return this.start().debounce(0.5);
  }

  public Trigger goSlow() {
    return this.leftTrigger();
  }

  public Trigger lockHeadingForDefense() {
    return this.leftBumper();
  }

  public Trigger stowIntake(){
    return this.povDown();
  }

  public Trigger deployIntake(){
    return this.povUp();
  }

  public Trigger agitateIntake(){
    return this.povLeft();
  }

  public Trigger fuelDump() {
    return this.povRight();
  }

  public Trigger runIntake(){
    return this.a();
  }

  public Trigger reverseIndexer(){
    return this.y();
  }

  public Trigger runIntakeReverse(){
    return this.b();
  }

  public Trigger shootFuel(){
    return this.rightTrigger();
  }

  // public Trigger robotRelative() {
  //   return this.a();
  // }

}