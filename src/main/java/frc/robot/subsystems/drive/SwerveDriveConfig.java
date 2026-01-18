// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class SwerveDriveConfig {
  public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(2);
  public static final double SDS_L2_DRIVE_GEARING = 6.75;
  public static final double kDriveEncoderPositionConversionFactor = (2 * Math.PI * kDriveWheelRadiusMeters)
      / SDS_L2_DRIVE_GEARING;
  public static final double kDriveEncoderVelocityConversionFactor = kDriveEncoderPositionConversionFactor / 60.0;

  public static final Translation2d kFrontLeftModuleOffset = new Translation2d(Units.inchesToMeters(11.25),
      Units.inchesToMeters(11.75));
  public static final Translation2d kFrontRightModuleOffset = new Translation2d(Units.inchesToMeters(11.25),
      Units.inchesToMeters(-11.75));
  public static final Translation2d kRearLeftModuleOffset = new Translation2d(Units.inchesToMeters(-11.25),
      Units.inchesToMeters(11.75));
  public static final Translation2d kRearRightModuleOffset = new Translation2d(Units.inchesToMeters(-11.25),
      Units.inchesToMeters(-11.75));

  // April Tags
  public static final double VISION_X_P = 1.25;
  public static final double VISION_Y_P = 2;
  public static final double VISION_REEF_TARGET_X_DISTANCE_METERS = 0.15;
  public static final double VISION_REEF_TARGET_Y_DISTANCE_METERS = 0.02;
  public static final double VISION_CORAL_STATION_TARGET_X_DISTANCE_METERS = 0.7;
  public static final double VISION_CORAL_STATION_TARGET_Y_DISTANCE_METERS = -0.18;

  // individual offsets after calibrating each module
  public static final double kFrontLeftPivotOffsetDegrees = 225;
  public static final double kFrontRightPivotOffsetDegrees = 135;
  public static final double kRearLeftPivotOffsetDegrees = 315;
  public static final double kRearRightPivotOffsetDegrees = 45;

  public static final double DRIVE_MODULE_P = 0.01;
  public static final double DRIVE_MODULE_I = 0;
  public static final double DRIVE_MODULE_D = 0;
  public static final double DRIVE_MODULE_KS = 0.11408;
  public static final double DRIVE_MODULE_KV = 3.2717;
  public static final double DRIVE_MODULE_KA = 0.17904;

  public static final double DRIVE_MODULE_PIVOT_P = 0.007;
  public static final double DRIVE_MODULE_PIVOT_I = 0.0;
  public static final double DRIVE_MODULE_PIVOT_D = 0.0001;

  public static final double DRIVE_ANGLE_P = 0.005;
  public static final double DRIVE_ANGLE_I = 0;
  public static final double DRIVE_ANGLE_D = 0;

  public static final double DRIVE_X_P = 0.3;
  public static final double DRIVE_X_I = 0;
  public static final double DRIVE_X_D = 0;

  public static final double DRIVE_Y_P = 0.3;
  public static final double DRIVE_Y_I = 0;
  public static final double DRIVE_Y_D = 0;

  public static final double DRIVE_PATHING_TRANSLATION_P = 4.0;
  public static final double DRIVE_PATHING_TRANSLATION_I = 0.0;
  public static final double DRIVE_PATHING_TRANSLATION_D = 0.3;

  public static final double DRIVE_PATHING_ROTATION_P = 1.5;
  public static final double DRIVE_PATHING_ROTATION_I = 0.0;
  public static final double DRIVE_PATHING_ROTATION_D = 0.8;
}