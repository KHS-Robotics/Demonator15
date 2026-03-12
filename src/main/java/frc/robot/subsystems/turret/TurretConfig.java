package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConfig {

    protected final class HoodConfig {
        public static final double kHoodP = 0.015;
        public static final double kHoodI = 0.0005;
        public static final double kHoodD = 0.03;
        
        public static final double kHoodEncoderPositionConversionFactor = 3.791;
        public static final double kHoodEncoderVelocityConversionFactor = kHoodEncoderPositionConversionFactor / 60.0;
        
        public static final double kMinSoftLimit = 0;
        public static final double kMaxSoftLimit = 40;

        public static final double kMaxAbsoluteDegrees = 39.986;
        public static final double kMaxAbsoluteVolts = 1.577;
        public static final double kMinAbsoluteDegrees = 0.0;
        public static final double kMinAbsoluteVolts = 1.930;

        public static final double kHoodAnalogPositionConversionFactor = 
        (WaistConfig.kMinAbsoluteDegrees - WaistConfig.kMaxAbsoluteDegrees) / (WaistConfig.kMinAbsoluteVolts - WaistConfig.kMaxAbsoluteVolts);
        public static final double kHoodAnalogVelocityConversionFactor = kHoodAnalogPositionConversionFactor / 60.0;
        public static final double kAbsoluteOffset = -331.531;

        //I think we will need a conversion for rotations of motor to how many degrees the hood will change
        public static final double kRotationsToDegreesConversion = 3.791;
    }

    protected final class SpitterConfig {
        public static final double kSpitterP = 0.0025;
        public static final double kSpitterI = 0.000001;
        public static final double kSpitterD = 0.04;

        public static final double kSpitterRPM = 5000;
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

        //left and right from blue drive team perspective for both
        public static final Translation2d kPassingPositionBlueLeft = new Translation2d(1,6.5);
        public static final Translation2d kPassingPositionBlueRight = new Translation2d(1,1.5);
        public static final Translation2d kPassingPositionRedLeft = new Translation2d(15,6.5);
        public static final Translation2d kPassingPositionRedRight = new Translation2d(15,1.5);

        public static final double kGravity = 9.8;

        public static final double kShooterVelocity = 8;

        //offset between navX zero and turret zero
        public static final double kDegreeOffsetOfTurret = 180;
        public static final double kTurretDistanceToNavX = 0;
        //in meters, at ground level
        public static final double kHeightBetweenShooterAndHub = 1.5;
        //presets: 8 shooter velocity - 4.88 max distance
        // 8.5 shooter velocity - 5.95 max distance
        // 8.25 shooter velocity - 5.38 max distance
        // 8.1 shooter velocity - 5.11 max distance
        public static final double kMaxDistance = 4.88;
    }


    protected final class WaistConfig {
        public static final double kAimerWaistP = 0.01;
        public static final double kAimerWaistI = 0.000075;
        public static final double kAimerWaistD = 0.05;
        
        public static final double kWaistRadiansOffset = Math.PI;

        public static final double kWaistEncoderPositionConversionFactor = 14.21;
        public static final double kWaistEncoderVelocityConversionFactor = kWaistEncoderPositionConversionFactor / 60.0;
        
        public static final double kMaxAbsoluteDegrees = -100.147;
        public static final double kMaxAbsoluteVolts = 1.934;
        public static final double kMinAbsoluteDegrees = 20.300;
        public static final double kMinAbsoluteVolts = 1.229;

        public static final double kWaistAnalogPositionConversionFactor = 
        (WaistConfig.kMinAbsoluteDegrees - WaistConfig.kMaxAbsoluteDegrees) / (WaistConfig.kMinAbsoluteVolts - WaistConfig.kMaxAbsoluteVolts);
        public static final double kWaistAnalogVelocityConversionFactor = kWaistAnalogPositionConversionFactor / 60.0;
        public static final double kAbsoluteOffset = -229.647;

        public static final double kMinSoftLimit = -100;
        public static final double kMaxSoftLimit = 22;
    }
}
