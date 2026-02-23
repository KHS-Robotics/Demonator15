package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConfig {

    protected final class HoodConfig {
        public static final double kHoodP = 0.02;
        public static final double kHoodI = 0.0005;
        public static final double kHoodD = 0;

        public static final double kHoodMaxAngle = 45;
        public static final double kHoodMinAngle = 0;
        
        public static final double kHoodEncoderPositionConversionFactor = 3.791;
        public static final double kHoodEncoderVelocityConversionFactor = kHoodEncoderPositionConversionFactor / 60.0;
        
        public static final double kMinSoftLimit = 0;
        public static final double kMaxSoftLimit = 45;

        //I think we will need a conversion for rotations of motor to how many degrees the hood will change
        public static final double kRotationsToDegreesConversion = 3.791;

        public static final double kHoodDegreesOffset = 0;
    }

    protected final class SpitterConfig {
        public static final double kSpitterP = 0;
        public static final double kSpitterI = 0;
        public static final double kSpitterD = 0;
    }

    protected final class TurretFieldAndRobotInfo {
        //in meters
        public static final double kRedHubPositionX = 11.92;
        public static final double kRedHubPositionY = 4.02;
        public static final Translation2d kRedHubPositionOnField = new Translation2d(kRedHubPositionX, kRedHubPositionY);

        //in meters
        public static final double kBlueHubPositionX = 4.65;
        public static final double kBlueHubPositionY = 4.02;
        public static final Translation2d kBlueHubPositionOnField = new Translation2d(kBlueHubPositionX, kBlueHubPositionY);

        public static final double kGravity = 9.8;

        public static final double kShooterVelocity = 9;

        //offset between navX zero and turret zero
        public static final double kDegreeOffsetOfTurret = 0;
        public static final double kTurretDistanceToNavX = 0;
        //in meters, at ground level
        public static final double kHeightBetweenShooterAndHub = 1.8;
    }


    protected final class WaistConfig {
        public static final double kAimerWaistP = 0.01;
        public static final double kAimerWaistI = 0.000075;
        public static final double kAimerWaistD = 0.05;
        
        public static final double kWaistDegreesOffset = 0;

        public static final double kWaistEncoderPositionConversionFactor = 14.21;
        public static final double kWaistEncoderVelocityConversionFactor = kWaistEncoderPositionConversionFactor / 60.0;

        public static final double kMinSoftLimit = -120;
        public static final double kMaxSoftLimit = 120;
    }
}
