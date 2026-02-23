package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

public class Turret extends SubsystemBase{
    private final Belt belt = new Belt();
    private final Hood hood = new Hood();
    private final Waist waist = new Waist();
    private final Kicker kicker = new Kicker();
    private final Spitter spitter = new Spitter();

    public Turret() {
        SmartDashboard.putData(this);
    }

    public void periodic() {
        RobotContainer.kField.getObject("waistAiming").setPose(
            RobotContainer.kSwerveDrive.getPose().getX(),
            RobotContainer.kSwerveDrive.getPose().getY(),
            getWaistFinalRotation(getCurrentHubPosition())
            );
    }

    public void stop() {
        hood.stop();
        waist.stop();
        spitter.stop();
        kicker.stop();
        belt.stop();
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        cmd.addRequirements(hood, waist, kicker, spitter, belt);
        return cmd.withName("StopTurret");
    }

    public Command shoot() {
        var cmd = spitter.startEnd(spitter::start, spitter::stop);
        return cmd;
    }

    public Command kick() {
        var cmd = kicker.startEnd(kicker::start, kicker::stop);
        return cmd;
    }

    public Command feed() {
        var cmd = belt.startEnd(belt::start, belt::stop);
        return cmd;
    }

    public Command reload() {
        var startKicker = kicker.startCommand();
        var startBelt = belt.startCommand();
        var cmd = startKicker.alongWith(startBelt);
        return cmd;
    }

    public Command shootContinuously() {
        var startSpitter = spitter.startCommand();
        var startKicker = kicker.startCommand();
        var startBelt = belt.startCommand();
        var cmd = startSpitter.alongWith(startKicker).alongWith(startBelt);
        return cmd;
    }

    private Translation2d getCurrentHubPosition() {
        Translation2d hubPosition;
        var alliance = DriverStation.getAlliance();
        if ( alliance.isPresent() && alliance.get() == Alliance.Red) {
            hubPosition = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionOnField;
        //if the alliance is null, it is auto set to blue variables
        }else {
            hubPosition = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionOnField;
        }
        return hubPosition;
    }

    private double getAngleToPosition() {
        return getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(), getCurrentHubPosition());
    }

    private double getAngleToPosition(Translation2d fromPose, Translation2d toPose) {
        double yDistanceTo = toPose.getY() - fromPose.getY();
        double xDistanceTo = toPose.getX() - fromPose.getX();
        
        var angleToHub =  Math.atan2(yDistanceTo, xDistanceTo);
        return angleToHub;
    }


    private double getDistanceToHub(Translation2d pose) {
        double distanceToHub = pose.getDistance(getCurrentHubPosition());
        return distanceToHub;
    }

    private double getDistanceToHub() {
        return getDistanceToHub(RobotContainer.kSwerveDrive.getPose().getTranslation());
    }



    private double solvePitch(double distance) {
        //uses newtons method to itterate until it finds a zero (or something close enough) of the original physics function
        //the original function shows where the angle needs to be to hit a set point, in this case the hub
        //the function has two answers (where x = 0) that give us an angle that hits the hub, we take the higher one by calculating using a higher guess for the angle
        double theta = Math.toRadians(88); 
        double g = TurretConfig.TurretFieldAndRobotInfo.kGravity;
        double d = distance;
        double v = TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        double h = TurretConfig.TurretFieldAndRobotInfo.kHeightBetweenShooterAndHub;
        for (int i = 0; i < 20; i++) {
            double func = d * Math.tan(theta) - (g * d * d ) / (2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) - h;
            if (Math.abs(func) < 0.001) {
                return theta;
            }
            double derivative = (d * (1 / Math.cos(theta)) * (1 / Math.cos(theta)) - (  4 * g * d * d * (v * Math.cos(theta)) * v * Math.sin(theta)) / ((2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) * (2 * (v * Math.cos(theta)) * (v * Math.cos(theta)))) );
            double step = func / derivative;
            theta = theta - step;
        }
        return theta;
    }


        //var velocityX = RobotContainer.kNavx.getVelocityX();
        //var velocityY = RobotContainer.kNavx.getVelocityY();
        //use these variable when getting the robots radial

        //use different things for the turret radial
    private double getRadialVelocityForPhantom(double velocityX, double velocityY, Translation2d position) {
        //getting the radial velocity from a phantom position we got from tangential velocity calcs to account for different radial distances
        //radial velocity is in reference to the velocity of the robot towards the hub
        // all velocity is in meters per second
        Translation2d robotPosition = position;

        var alliance = DriverStation.getAlliance();
        Translation2d radialVector;
        double rX;
        double rY;
        if (  alliance.isPresent() && alliance.get() == Alliance.Red) {
            rX = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY - robotPosition.getY();
        //if the alliance is null, it is auto set to blue variables
        }else {
            rX = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY - robotPosition.getY();
        }
        radialVector = new Translation2d(rX, rY);
        //distance to hub
        var radialVectorNorm = radialVector.getNorm();
        Translation2d radialUnitVector;
        if (radialVectorNorm != 0) {
            radialUnitVector = new Translation2d(radialVector.getX() / radialVectorNorm, radialVector.getY() / radialVectorNorm);
        }else {
            radialUnitVector = new Translation2d(0,0);
        }
        double radVelocity = (velocityX * radialUnitVector.getX() + velocityY * radialUnitVector.getY());
        return radVelocity;
    }

    private double getTangentialVelocity(double velocityX, double velocityY) {
        //tangential velocity is in reference to velocity of the robot around the hub in le circle
        // we just use the radial velocity code and then rotate the vector
        Translation2d robotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        var alliance = DriverStation.getAlliance();
        Translation2d radialVector;
        double rX;
        double rY;
        if (  alliance.isPresent() && alliance.get() == Alliance.Red) {
            rX = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY - robotPosition.getY();
        //if the alliance is null, it is auto set to blue variables
        }else {
            rX = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY - robotPosition.getY();
        }
        radialVector = new Translation2d(rX, rY);
        //distance to hub
        var radialVectorNorm = radialVector.getNorm();
        Translation2d radialUnitVector;
        if (radialVectorNorm != 0) {
            radialUnitVector = new Translation2d(radialVector.getX() / radialVectorNorm, radialVector.getY() / radialVectorNorm);
        }else {
            radialUnitVector = new Translation2d(0,0);
        }
        Translation2d tangentialUnitVector = radialUnitVector.rotateBy(Rotation2d.kCCW_90deg);
        double tagVelocity = (velocityX * tangentialUnitVector.getX() + velocityY * tangentialUnitVector.getY());
        return tagVelocity;
    }

    /**
     * @return the linear velocity of the turret based on the rotation of the robot
     */
    private Translation2d getTurretRotationalToLinearVelocity() {
        Rotation2d theta = new Rotation2d(Math.toRadians(RobotContainer.kSwerveDrive.getPose().getRotation().getDegrees() + 90 + TurretConfig.TurretFieldAndRobotInfo.kDegreeOffsetOfTurret));
        var magnitude = new Rotation2d(RobotContainer.kSwerveDrive.getChassisSpeeds().omegaRadiansPerSecond).getDegrees() * TurretConfig.TurretFieldAndRobotInfo.kTurretDistanceToNavX;
        Translation2d RotationalVelocity = new Translation2d(magnitude, theta);
        return RotationalVelocity;
    }

    /**
    * @return the fake robot position for position calcs based on tangential velocity
    */
    private Translation2d getPhantomRobotPosition() {
        //just using tangential velocity calcs
        double tangVelocity = getTangentialVelocity(RobotContainer.kSwerveDrive.getChassisSpeeds().vxMetersPerSecond, RobotContainer.kSwerveDrive.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d angleOfTang = new Rotation2d(getAngleToPosition() + (Math.PI / 2));
        var pitchAngle = solvePitch(getDistanceToHub());
        double T = getDistanceToHub() / (TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity * Math.cos(pitchAngle));
        var rotationalVelocity = getTurretRotationalToLinearVelocity();
        double robotRotationalTangVelocity = getTangentialVelocity(rotationalVelocity.getX(), rotationalVelocity.getY());
        double phantomYawDistance = T * (tangVelocity + robotRotationalTangVelocity);
        Translation2d phantomYawVector = new Translation2d(phantomYawDistance, angleOfTang);
        //translation 2ds just happen to be a canvinient way to use a two variable variable, some translation2ds actually represent velocities
        Translation2d phantomYawPosition = phantomYawVector.plus(RobotContainer.kSwerveDrive.getPose().getTranslation());

        return phantomYawPosition;
    }

    private double aimWaistAngle(Translation2d towards) {
        
        var angle = getWaistFinalRotation(towards).getDegrees();
        return angle;
    }
    
    private double aimWaistAngle() {
        return aimWaistAngle(getCurrentHubPosition());
    }

    private Rotation2d getWaistFinalRotation(Translation2d towards) {
        Translation2d phantomRobotPosition = getPhantomRobotPosition();
        //the adding of the yaw is to account for the rotation of the robot
        double angle =  getAngleToPosition(phantomRobotPosition, towards) - RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
        //clamping the value because our turret only goes 240(?) degrees
        angle = MathUtil.clamp(Math.toDegrees(angle) % 360, -120, 120);
        Rotation2d angleRotation = new Rotation2d(angle);
        //+ TurretConfig.WaistConfig.kWaistDegreesOffset);
        return angleRotation;
    }

    private double getHoodAimFinalAngle(Translation2d towards) {
        //linear calcs
        Translation2d phantomRobotPosition = getPhantomRobotPosition();
        double radialVelocity = getRadialVelocityForPhantom(RobotContainer.kSwerveDrive.getChassisSpeeds().vxMetersPerSecond, RobotContainer.kSwerveDrive.getChassisSpeeds().vyMetersPerSecond, phantomRobotPosition);
        Rotation2d angleOfRad = new Rotation2d(getAngleToPosition(phantomRobotPosition, towards));
        double T = getDistanceToHub(phantomRobotPosition) / TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        //rotational calcs
        var rotationalVelocity = getTurretRotationalToLinearVelocity();
        double robotRotationalRadVelocity = getTangentialVelocity(rotationalVelocity.getX(), rotationalVelocity.getY());

        double phantomPitchDistance = T * radialVelocity + robotRotationalRadVelocity;
        Translation2d phantomPitchVector = new Translation2d(phantomPitchDistance, angleOfRad);
        Translation2d phantomPitchPosition = phantomPitchVector.plus(phantomRobotPosition);

        double distanceToPoint = phantomPitchPosition.getDistance(towards);
        var angle = 90 - Math.toDegrees(solvePitch(distanceToPoint));
        //add radial velocity calcs
        //clamp to the physical limits of our hood
        angle = MathUtil.clamp(angle, 0.0, 45.0); 
        //this will be part of the relative / absolute hybrid incorporation   + TurretConfig.HoodConfig.kHoodDegreesOffset;
        return angle;
    }

    
    private double aimHoodAngle(Translation2d towards) {
        double angle = getHoodAimFinalAngle(towards);
        return angle;
    }

    private double aimHoodAngle() {
        return aimHoodAngle(getCurrentHubPosition());
    }

    public Command aimTowardsHubWithVelocity() {
        var aimWaist = hood.setAngleCommand(aimWaistAngle());
        var aimHood = waist.setDegreesCommand(aimHoodAngle());
        var cmd = aimWaist.alongWith(aimHood);
        return cmd;
    }

    public Command aimTowardsHub() {
        var aimWaist = hood.aimHoodSimple(getCurrentHubPosition());
        var aimHood = waist.aimWaistSimple(getCurrentHubPosition());
        var cmd = aimWaist.alongWith(aimHood);
        return cmd;
    }

    public Command aimAndShootTowardsHub() {
        var aimWaist = hood.aimHoodSimple(getCurrentHubPosition());
        var aimHood = waist.aimWaistSimple(getCurrentHubPosition());
        var startSpitter = spitter.startCommand();
        var startKicker = kicker.startCommand();
        var startBelt = belt.startCommand();
        var cmd = aimWaist.alongWith(aimHood.alongWith(startBelt).alongWith(startKicker).alongWith(startSpitter));
        return cmd;
    }

    public Command aimAndShootTowardsHubWithVelocity() {
        var aimWaist = hood.setAngleCommand(aimWaistAngle());
        var aimHood = waist.setDegreesCommand(aimHoodAngle());
        var startSpitter = spitter.startCommand();
        var startKicker = kicker.startCommand();
        var startBelt = belt.startCommand();
        var cmd = aimWaist.alongWith(aimHood.alongWith(startBelt).alongWith(startKicker).alongWith(startSpitter));
        return cmd;
    }

    public Command goToSetHoodAngle() {
        var cmd = hood.setAngleCommand(20);
        return cmd;
    }
    
    public Command goToSetWaistAngle() {
        var cmd = waist.setDegreesCommand(0);
        return cmd;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.setSafeState(this::stop);
        builder.addDoubleProperty("testHoodCalcs",() -> getHoodAimFinalAngle(getCurrentHubPosition()), null);
    }
}
    //for getting the position and rotation (which we will probably want to do in hood and waist), use our humble kNavx
    //
    //    var navXRotation = RobotContainer.kNavx.getYaw();
    //    var navXVelocityX = RobotContainer.kNavx.getVelocityX();
    //    var navXVelocityY = RobotContainer.kNavx.getVelocityY();
    //    var swerveDrivePosition = RobotContainer.kSwerveDrive.getPose();
    // T = distanceToHub / v;
    // phantomPitchPosition = T * getRadialVelocity();