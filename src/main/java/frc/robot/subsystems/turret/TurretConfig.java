package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConfig {

    protected final class HoodConfig {
        public static final double kHoodP = 0;
        public static final double kHoodI = 0;
        public static final double kHoodD = 0;

        public static final double kHoodMaxAngle = 45;
        public static final double kHoodMinAngle = 0;

        //I think we will need a conversion for rotations of motor to how many degrees the hood will change
        public static final double kRotationsToDegreesConversion = 0.0;

        public static final double kHoodDegreesOffset = 0;
    }

    protected final class TurretFieldAndRobotInfo {
        //in meters
        public static final double kRedHubPositionX = 0.0;
        public static final double kRedHubPositionY = 0.0;
        public static final Translation2d kRedHubPositionOnField = new Translation2d(kRedHubPositionX, kRedHubPositionY);

        //in meters
        public static final double kBlueHubPositionX = 0.0;
        public static final double kBlueHubPositionY = 0.0;
        public static final Translation2d kBlueHubPositionOnField = new Translation2d(kBlueHubPositionX, kBlueHubPositionY);

        public static final double kGravity = 9.8;

        public static final double kShooterVelocity = 9;
        //in meters, at ground level
        public static final double kHeightBetweenShooterAndHub = 1.8;
    }


    protected final class WaistConfig {
        public static final double kAimerWaistP = 0;
        public static final double kAimerWaistI = 0;
        public static final double kAimerWaistD = 0;
        
        public static final double kWaistDegreesOffset = 0;
    }
}
