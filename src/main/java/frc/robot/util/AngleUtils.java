package frc.robot.util;

/**
 * Shared angle normalization for mechanisms that report degrees outside ±180°.
 */
public final class AngleUtils {
  private AngleUtils() {}

  /**
   * Wraps an angle to the principal range {@code [-180, 180]} degrees.
   *
   * @param angleDegrees input angle in degrees
   * @return equivalent angle in [-180, 180]
   */
  public static double normalizeDegrees(double angleDegrees) {
    double angle = angleDegrees;
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
