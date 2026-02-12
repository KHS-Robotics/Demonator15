/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// VALUES IMPORTED FROM D14, SUBJECT TO CHANGE

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {
  // Joysticks
  public static final int XBOX_PORT = 0;
  public static final int OPERATOR_STICK_PORT = 1;
  
  // Swerve pivot motors
  public static final int FRONT_LEFT_PIVOT = 10;
  public static final int FRONT_RIGHT_PIVOT = 8;
  public static final int REAR_LEFT_PIVOT = 18;
  public static final int REAR_RIGHT_PIVOT = 20;

  // Swerve drive motors
  public static final int FRONT_LEFT_DRIVE = 11;
  public static final int FRONT_RIGHT_DRIVE = 9;
  public static final int REAR_LEFT_DRIVE = 19;
  public static final int REAR_RIGHT_DRIVE = 1;

  // Swerve pivot encoders
  public static final int FRONT_LEFT_PIVOT_ENCODER = 12;
  public static final int FRONT_RIGHT_PIVOT_ENCODER = 22;
  public static final int REAR_LEFT_PIVOT_ENCODER = 32;
  public static final int REAR_RIGHT_PIVOT_ENCODER = 42;

//climber motors
  public static final int CLIMBER_ID = -1;

//indexer motors
  public static final int HOPPER_MOTOR_ID = -2;

//intake motors
  public static final int INTAKE_DEPLOYER_LEADER_ID = -3;
  public static final int INTAKE_DEPLOYER_FOLLOWER_ID = -4;
  public static final int INTAKE_GRABBY_WHEELS_ID = -5;

//turret motors
  public static final int TURRET_AIMER_HOOD_ID = 41;
  public static final int TURRET_AIMER_WAIST_ID = 43;
  public static final int TURRET_SPITTER_LEADER_ID = 42;
  public static final int TURRET_KICKER_MOTOR_ID = 44;
  


//LEDs over PWM
  public static final int LED_PORT = 0;

//one climber motor - climber
//eight swerve motors
//one indexer motor - hopper
//one deployer motor - deployer
//one grabby wheel motor - grabber
//four wheel motors for the shooter:
//indexer -> tower motor - belt
//pre-shooter motor - kicker
//two shooter motors - spitter lead and spitter follower
//two motors for aiming the shooter - hood (pitch) and waist (yaw)
}