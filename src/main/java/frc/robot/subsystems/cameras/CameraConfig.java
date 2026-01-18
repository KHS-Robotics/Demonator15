package frc.robot.subsystems.cameras;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.cameras.DemonLimelightCamera.LimelightPoseEstimateAlgorithm;

public final class CameraConfig {
    //Blue alliance AprilTags

    // Blue alliance hub AprilTags
    public static final ArrayList<Integer> kBlueAllianceHubFiducialIds = new ArrayList<>();

    //Blue alliance trench AprilTags
    public static final ArrayList<Integer> kBlueAllianceTrenchFiducialIds = new ArrayList<>();

    // Blue alliance outpost AprilTags
    public static final ArrayList<Integer> kBlueAllianceOutpostFiducialIds = new ArrayList<>();

    // Blue alliance tower AprilTags
    public static final ArrayList<Integer> kBlueAllianceTowerFiducialIds = new ArrayList<>();

    //Red alliance AprilTags

    // Red alliance hub AprilTags
    public static final ArrayList<Integer> kRedAllianceHubFiducialIds = new ArrayList<>();

    //Red alliance trench AprilTags
    public static final ArrayList<Integer> kRedAllianceTrenchFiducialIds = new ArrayList<>();

    // Red alliance outpost AprilTags
    public static final ArrayList<Integer> kRedAllianceOutpostFiducialIds = new ArrayList<>();

    // Red alliance tower AprilTags
    public static final ArrayList<Integer> kRedAllianceTowerFiducialIds = new ArrayList<>();

    static{
        //Red alliance AprilTag values
        kRedAllianceHubFiducialIds.add(2);
        kRedAllianceHubFiducialIds.add(3);
        kRedAllianceHubFiducialIds.add(4);
        kRedAllianceHubFiducialIds.add(5);
        kRedAllianceHubFiducialIds.add(8);
        kRedAllianceHubFiducialIds.add(9);
        kRedAllianceHubFiducialIds.add(10);
        kRedAllianceHubFiducialIds.add(11);

        kRedAllianceTrenchFiducialIds.add(1);
        kRedAllianceTrenchFiducialIds.add(6);
        kRedAllianceTrenchFiducialIds.add(7);
        //6-7!!!!! xD lol
        kRedAllianceTrenchFiducialIds.add(12);

        kRedAllianceOutpostFiducialIds.add(13);
        kRedAllianceOutpostFiducialIds.add(14);

        kRedAllianceTowerFiducialIds.add(15);
        kRedAllianceTowerFiducialIds.add(16);

        //Blue alliance AprilTag Values
        kBlueAllianceHubFiducialIds.add(18);
        kBlueAllianceHubFiducialIds.add(19);
        kBlueAllianceHubFiducialIds.add(20);
        kBlueAllianceHubFiducialIds.add(21);
        kBlueAllianceHubFiducialIds.add(24);
        kBlueAllianceHubFiducialIds.add(25);
        kBlueAllianceHubFiducialIds.add(26);
        kBlueAllianceHubFiducialIds.add(27);

        kBlueAllianceTrenchFiducialIds.add(17);
        kBlueAllianceTrenchFiducialIds.add(22);
        kBlueAllianceTrenchFiducialIds.add(23);
        kBlueAllianceTrenchFiducialIds.add(28);

        kBlueAllianceOutpostFiducialIds.add(29);
        kBlueAllianceOutpostFiducialIds.add(30);

        kBlueAllianceTowerFiducialIds.add(31);
        kBlueAllianceTowerFiducialIds.add(32);
    }


    

  public class PhotonVisionConfig {
    /** The nickname of the camera (found in the PhotonVision UI). */
    public static final String PhotonCameraName = "PhotonCamera";
    /**
     * Transform3d from the center of the robot to the camera mount position (ie,
     * robot ➔ camera) in the <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     * Coordinate System</a>.
     * <p>
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-a-photonposeestimator
     */
    public static final Transform3d RobotToPhotonCamera = new Transform3d(Units.inchesToMeters(10.5),
        Units.inchesToMeters(-7), Units.inchesToMeters(16.5), new Rotation3d(0, Math.toRadians(20), 0));
    
    /**
     * The layout of the AprilTags on the field.
     * <p>
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#creating-an-apriltagfieldlayout
     * <p>
     * https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html
     */
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);
  }

  public class LimelightConfig {
    /** The name of the camera from the UI. */
    public static final String LimelightCameraName = "LimelightCamera";
    /** The pose estimation algorithm to use, */
    public static final LimelightPoseEstimateAlgorithm kPoseAlgorithm = LimelightPoseEstimateAlgorithm.Megatag2;
    /**
     * Transform3d from the center of the robot to the camera mount position (ie,
     * robot ➔ camera) in the <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     * Coordinate System</a>.
     * <p>
     * <b>This must be configured in the Limelight UI too under 3-D.</b>
     */
    public static final Transform3d kRobotToLimelightCamera = new Transform3d(Units.inchesToMeters(-10),
        Units.inchesToMeters(6), Units.inchesToMeters(30), new Rotation3d(0, Math.toRadians(6), Math.toRadians(180)));
  }
}