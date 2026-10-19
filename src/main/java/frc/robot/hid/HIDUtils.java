package frc.robot.hid;

/**
 * Utility functions for HIDs.
 */
public final class HIDUtils {
  /** Utility class with static methods only. */
  private HIDUtils() {
  }

  /**
   * Sensitivity control for the joystick that uses a cubic function to smooth out
   * the inputs instead of full linear control: <code> s*x^3 + (1-s)*x </code>
   * where s is the sensitivity and x is the input.
   * 
   * <p>
   * 
   * Example where sensitivity is set to 0.5:
   * https://www.wolframalpha.com/input?i=s*x%5E3+%2B+%281-s%29*x+where+s+%3D+0.5
   * 
   * @param input       the raw input from the joystick
   * @param sensitivity the sensitivity from [0, 1] where 0 is full linear and 1
   *                    is full cubic
   * @return the new dampened control value
   */
  public static double smoothInputWithCubic(double input, double sensitivity) {
    return sensitivity * Math.pow(input, 3) + (1 - sensitivity) * input;
  }

  /**
   * Deadband, cubic smoothing, and scaling for a single joystick axis.
   *
   * @param rawAxis raw axis value from the HID
   * @param deadband magnitudes at or below this are treated as zero
   * @param sensitivity cubic blend [0, 1]; 0 = linear, 1 = full cubic
   * @param maxOutput scale when axis is saturated (e.g. m/s or rad/s)
   * @return scaled output, or 0 inside the deadband
   */
  public static double scaledStickAxis(double rawAxis, double deadband, double sensitivity, double maxOutput) {
    if (Math.abs(rawAxis) <= deadband) {
      return 0.0;
    }
    return smoothInputWithCubic(rawAxis, sensitivity) * maxOutput;
  }
}