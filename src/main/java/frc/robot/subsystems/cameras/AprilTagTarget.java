package frc.robot.subsystems.cameras;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagTarget {
  public final Pose3d position;
  public final PhotonTrackedTarget tag;
  public final int id;

  public AprilTagTarget(Pose3d position, PhotonTrackedTarget tag, int id) {
    this.position = position;
    this.tag = tag;
    this.id = id;
  }

  public double getOffetX() {
    var dist = tag.getBestCameraToTarget();
    return dist.getX();
  }

  public double getOffsetY() {
    var dist = tag.getBestCameraToTarget();
    return dist.getY();
  }

  public double getTargetAngle() {
    // add 180 degrees to get blue side relative angles instead of red side relative
    var angle = normalizeAngle(Math.toDegrees(position.getRotation().getAngle()) + 180);
    return angle;
  }

  @Override
  public String toString() {
    return "ID = " + id + " :: X = " + getOffetX() + " :: Y = " + getOffsetY() + " :: Theta = " + getTargetAngle();
  }

  /**
   * Clamps an angle to be from [-180, 180]
   * 
   * @param angle the angle to clamp
   * @return the newly clamped angle from [-180, 180]
   */
  private static double normalizeAngle(double angle) {
    if (angle > 0) {
      angle %= 360;
      if (angle > 180) {
        angle -= 360;
      }
    } else if (angle < 0) {
      angle %= -360;
      if (angle < -180) {
        angle += 360;
      }
    }
    return angle;
  }
}