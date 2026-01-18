// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.cameras.CameraConfig.PhotonVisionConfig;

/**
 * Encapsulates a camera using Photon Vision.
 * 
 * <p>
 * 
 * <a href=
 * "https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/robot-pose-estimator.html">
 * AprilTags
 * </a>
 * 
 * <p>
 * 
 * <a href=
 * "https://docs.photonvision.org/en/v2025.1.1/docs/reflectiveAndShape/thresholding.html">
 * GamePiece
 * </a>
 */
public class DemonPhotonCamera extends SubsystemBase {
  private PhotonPipelineMode currentPipelineMode;
  private Optional<PhotonPoseUpdate> aprilTagUpdate = Optional.empty();
  private Optional<List<PhotonTrackedTarget>> gamePieceTargets = Optional.empty();
  private Optional<PhotonTrackedTarget> bestGamePieceTarget = Optional.empty();
  private boolean enableAprilTagUpdates = true;
  
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new camera to track AprilTags using Photon Vision.
   * 
   * @param cameraName   The nickname of the camera (found in the
   *                     PhotonVision UI)
   * @param cameraOffset Transform3d from the center of the robot to the
   *                     camera mount position (ie, robot ➔ camera) in the
   *                     <a href=
   *                     "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
   *                     Coordinate System</a>
   */
  public DemonPhotonCamera(String cameraName, Transform3d cameraOffset) {
    super("Cameras/" + cameraName);

    camera = new PhotonCamera(cameraName);

    poseEstimator = new PhotonPoseEstimator(PhotonVisionConfig.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraOffset);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    setCloseRangeAprilTagMode();

    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.addBooleanProperty("AprilTagEnabled", () -> enableAprilTagUpdates,
        (enable) -> enableAprilTagUpdates = enable);
    builder.addBooleanProperty("HasAprilTagUpdate", () -> aprilTagUpdate.isPresent(), null);
    builder.addBooleanProperty("HasGamePieceTargets", () -> gamePieceTargets.isPresent(), null);
    builder.addBooleanProperty("HasBestGamePieceTarget", () -> bestGamePieceTarget.isPresent(), null);
    builder.addIntegerProperty("NumAprilTags",
        () -> aprilTagUpdate.isPresent() ? aprilTagUpdate.get().cameraResult.getTargets().size() : 0, null);
    builder.addIntegerProperty("NumGamePiece", () -> gamePieceTargets.isPresent() ? gamePieceTargets.get().size() : 0, null);
    builder.addStringProperty("PipelineMode", () -> currentPipelineMode.toString(), null);
    builder.addBooleanProperty("Connected", () -> camera.isConnected(), null);
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    // make sure to call once per loop to get consistent results
    updateLatestVisionResults();
    clearVisionResultsOnDisconnect();
  }

  public Command hasVisibleTarget(Supplier<Integer> fiducialId) {
    return hasVisibleTarget(fiducialId, 0.3);
  }

  public Command hasVisibleTarget(Supplier<Integer> fiducialId, double debounceTime) {
    var cmd = new Command() {
      private Timer timer = new Timer();
      private int targetId = 0;
      private boolean seesTarget = false;
      
      @Override
      public void initialize() {
        targetId = fiducialId.get();
        timer.reset();
        timer.start();
      }

      @Override
      public void execute() {
        seesTarget = getAprilTagById(targetId).isPresent();
      }

      @Override
      public boolean isFinished() {
        if (!seesTarget) {
          timer.reset();
        }
        return timer.hasElapsed(debounceTime);
      }

      @Override
      public void end(boolean interrupted) {
        timer.stop();
      }
    };

    return cmd.withName("DemonPhotonCameraHasVisibleTarget");
  }

  /**
   * Command to apply an action to Photon pose updates.
   * 
   * @param action the action for the Photon pose update
   * @return a command to apply an action to Photon pose updates
   */
  public Command pollForPoseUpdates(Consumer<PhotonPoseUpdate> action) {
    var pollForPoseUpdatesCmd = run(() -> getLatestAprilTagResults().ifPresent((estimate) -> action.accept(estimate)));
    pollForPoseUpdatesCmd.setName(getName() + "_PollForPoseUpdates");
    return pollForPoseUpdatesCmd.ignoringDisable(true);
  }

  /**
   * Command to update the pipeline mode for the camera.
   * 
   * @param mode the mode to switch to
   * @return the command to update the pipeline mode for the camera
   */
  public Command setPipelineMode(PhotonPipelineMode mode) {
    return runOnce(() -> {
      switch (mode) {
        case kAprilTagsHighFPS:
          setCloseRangeAprilTagMode();
          break;
        case kAprilTagsHighResolution:
          setFarRangeAprilTagMode();
          break;
        case kGamePiece:
          setGamePieceMode();
          break;
        default:
          DriverStation.reportWarning("Unsupported Photon pipeline mode: " + mode, false);
      }
    });
  }

  /**
   * Sets the camera pipeline to AprilTag mode using lower resolution but higher
   * FPS.
   */
  public void setCloseRangeAprilTagMode() {
    if (currentPipelineMode == PhotonPipelineMode.kAprilTagsHighFPS)
      return;

    gamePieceTargets = Optional.empty();
    bestGamePieceTarget = Optional.empty();
    currentPipelineMode = PhotonPipelineMode.kAprilTagsHighFPS;
    camera.setPipelineIndex(currentPipelineMode.index);
  }

  /**
   * Sets the camera pipeline to AprilTag mode using higher resolution but lower
   * FPS.
   */
  public void setFarRangeAprilTagMode() {
    if (currentPipelineMode == PhotonPipelineMode.kAprilTagsHighResolution)
      return;

    gamePieceTargets = Optional.empty();
    bestGamePieceTarget = Optional.empty();
    currentPipelineMode = PhotonPipelineMode.kAprilTagsHighResolution;
    camera.setPipelineIndex(currentPipelineMode.index);
  }

  /**
   * Sets the camera pipeline to GamePiece mode.
   */
  public void setGamePieceMode() {
    if (currentPipelineMode == PhotonPipelineMode.kGamePiece)
      return;

    aprilTagUpdate = Optional.empty();
    currentPipelineMode = PhotonPipelineMode.kGamePiece;
    camera.setPipelineIndex(currentPipelineMode.index);
  }

  /**
   * Gets the latest processed update from photon vision for AprilTags.
   * 
   * @return the latest processed update from photon vision for AprilTags
   */
  public Optional<PhotonPoseUpdate> getLatestAprilTagResults() {
    return aprilTagUpdate;
  }

  /**
   * Gets the latest processed update from photon vision from
   * {@link org.photonvision.targeting.PhotonPipelineResult#getTargets()}.
   * 
   * @return all targets from the latest pipeline result
   */
  public Optional<List<PhotonTrackedTarget>> getLatestGamePieceTargets() {
    if (currentPipelineMode != PhotonPipelineMode.kGamePiece) {
      DriverStation.reportWarning("Attempting to get GamePiece results in AprilTag mode!", false);
    }

    return gamePieceTargets;
  }

  /**
   * Gets the latest processed update from photon vision from
   * {@link org.photonvision.targeting.PhotonPipelineResult#getBestTarget()}.
   * 
   * @return the best target from the latest pipeline result
   */
  public Optional<PhotonTrackedTarget> getLatestBestGamePieceTarget() {
    if (currentPipelineMode != PhotonPipelineMode.kGamePiece) {
      DriverStation.reportWarning("Attempting to get GamePiece results in AprilTag mode!", false);
    }

    return bestGamePieceTarget;
  }

  /**
   * Sets the camera to receive AprilTag updates or not for the odometry.
   * 
   * @param enableAprilTagUpdates true to enable AprilTag updates, false to
   *                              disable AprilTag updates
   */
  public void setEnableAprilTagUpdates(boolean enableAprilTagUpdates) {
    this.enableAprilTagUpdates = enableAprilTagUpdates;
  }

  public Optional<AprilTagTarget> getBestAprilTag(ArrayList<Integer> fiducialIds) {
    var results = getLatestAprilTagResults();
    if (results.isEmpty())
      return Optional.empty();
    
    var targetWithLargestArea = new PhotonTrackedTarget();
    var cameraResult = results.get().cameraResult;
    boolean hasTarget = false;
    for (var result : cameraResult.getTargets()) {
      if (result.area > targetWithLargestArea.area && (fiducialIds.isEmpty() || fiducialIds.contains(result.fiducialId))) {
        targetWithLargestArea = result;
        hasTarget = true;
      }
    }
    if(!hasTarget){
      return Optional.empty();
    }
    return getApriltagFromTarget(targetWithLargestArea);
  }

  public Optional<AprilTagTarget> getAprilTagById(int id) {
    var results = getLatestAprilTagResults();
    if (results.isEmpty())
      return Optional.empty();
    
    var cameraResult = results.get().cameraResult;
    for (var result : cameraResult.getTargets()) {
      if (result.fiducialId == id) {
        return getApriltagFromTarget(result);
      }
    }
    return Optional.empty();
  }

  private Optional<AprilTagTarget> getApriltagFromTarget(PhotonTrackedTarget target) {
    var targetPositionOpt = PhotonVisionConfig.kTagLayout.getTagPose(target.getFiducialId());
    if (targetPositionOpt.isEmpty())
      return Optional.empty();
    
    var targetPosition = targetPositionOpt.get();
    return Optional.of(new AprilTagTarget(targetPosition, target, target.getFiducialId()));
  }

  /**
   * Processes all current unread results.
   */
  private void updateLatestVisionResults() {
    // get unprocessed results from photon
    var unprocessedCameraResults = camera.getAllUnreadResults();

    if (!unprocessedCameraResults.isEmpty()) {
      // get latest result
      var cameraResult = unprocessedCameraResults.get(unprocessedCameraResults.size() - 1);
      
      // process based on selected pipeline mode
      switch (currentPipelineMode) {
        // same AprilTag process for both resolutions
        case kAprilTagsHighFPS:
        case kAprilTagsHighResolution:
          processAprilTagResult(cameraResult);
          break;
        case kGamePiece:
          processGamePieceResult(cameraResult);
          break;
        default:
          DriverStation.reportWarning("Unsupported Photon pipeline mode: " + currentPipelineMode, false);
      }
    }
  }

  /**
   * Clears all currently cached vision results when the camera is disconnected.
   */
  private void clearVisionResultsOnDisconnect() {
    if (!camera.isConnected()) {
      aprilTagUpdate = Optional.empty();
      gamePieceTargets = Optional.empty();
      bestGamePieceTarget = Optional.empty();
    }
  }

  /**
   * Processes the camera result for the AprilTag pipeline.
   * 
   * @param cameraResult the camera result from Photon
   */
  private void processAprilTagResult(PhotonPipelineResult cameraResult) {
    // check if AprilTag updates are enabled
    if (!enableAprilTagUpdates) {
      aprilTagUpdate = Optional.empty();
      return;
    }

    // process results from photon
    var photonPoseUpdate = poseEstimator.update(cameraResult);

    // check if processed result has an update from photon
    if (photonPoseUpdate.isPresent()) {
      // get the update from photon
      var photonPose = photonPoseUpdate.get();

      // calculate standard deviations and set the latest update
      var stdDevs = getEstimatedStdDevsForAprilTagResult(cameraResult, photonPose.estimatedPose.toPose2d());
      aprilTagUpdate = Optional.of(new PhotonPoseUpdate(cameraResult, photonPose, stdDevs));
    } else {
      // no update from photon
      aprilTagUpdate = Optional.empty();
    }
  }

  /**
   * Processes the camera result for the GamePiece pipeline.
   * 
   * @param cameraResult the camera result from Photon
   */
  private void processGamePieceResult(PhotonPipelineResult cameraResult) {
    // check if any visible targets
    if (cameraResult.hasTargets()) {
      gamePieceTargets = Optional.of(cameraResult.getTargets());
      bestGamePieceTarget = Optional.of(cameraResult.getBestTarget());
    } else {
      gamePieceTargets = Optional.empty();
      bestGamePieceTarget = Optional.empty();
    }
  }

  /**
   * Calcualtes the estimated standard deviations based on number of visible April
   * Tags and their distances from the camera.
   * 
   * @param result        the result from photon
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getEstimatedStdDevsForAprilTagResult(PhotonPipelineResult result, Pose2d estimatedPose) {
    try {
      if (result == null) {
        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      }

      var estStdDevs = VecBuilder.fill(0.5, 0.5, 999999999);
      var targets = result.getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }

      if (numTags == 0)
        return estStdDevs;

      avgDist /= numTags;

      // Decrease std devs if multiple targets are visible
      if (numTags > 1 && avgDist < 3.5)
        estStdDevs = VecBuilder.fill(0.2, 0.2, 999999999);

      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 3.5)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
    } catch (Exception exception) {
      exception.printStackTrace();
      DriverStation.reportError("Failed to calculate stddev for Photon.", false);
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
  }

  /**
   * Enumeration for Photon pipeline modes for the camera.
   */
  private static enum PhotonPipelineMode {
    /**
     * AprilTag detection mode for close range (lower resolution but higher FPS).
     */
    kAprilTagsHighFPS(0),

    /**
     * AprilTag detection mode for far range (higher resolution for but lower FPS).
     */
    kAprilTagsHighResolution(1),

    /**
     * GamePiece detection mode.
     */
    kGamePiece(2);

    /** The Photon pipeline index. */
    public final int index;

    /**
     * Enumeration for Photon pipeline modes for the camera.
     * 
     * @param index the Photon pipeline index
     */
    PhotonPipelineMode(int index) {
      this.index = index;
    }
  }

  /**
   * Holds all applicable information to a photon vision update.
   */
  public static class PhotonPoseUpdate {
    /** The camera result used for the update. */
    public final PhotonPipelineResult cameraResult;
    /** The estimated robot pose from the pose estimator. */
    public final EstimatedRobotPose estimatedRobotPose;
    /** The standard deviations to use for the vision update. */
    public final Matrix<N3, N1> stdDevs;

    /**
     * Holds all applicable information to a photon vision update.
     * 
     * @param cameraResult       the pipeline result from
     *                           {@link org.photonvision.PhotonCamera#getAllUnreadResults()}
     * @param estimatedRobotPose the estimated robot pose from
     *                           {@link org.photonvision.PhotonPoseEstimator#update(PhotonPipelineResult)}
     * @param stdDevs            the standard deviations for the vision update
     */
    private PhotonPoseUpdate(PhotonPipelineResult cameraResult, EstimatedRobotPose estimatedRobotPose,
        Matrix<N3, N1> stdDevs) {
      this.cameraResult = cameraResult;
      this.estimatedRobotPose = estimatedRobotPose;
      this.stdDevs = stdDevs;
    }
  }
}