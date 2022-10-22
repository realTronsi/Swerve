package frc.robot.utility;

public class MathPP {
  /**
   * 
   * @param a deg
   * @return input converted to radians
   */
  public static final double degToRad(double a) {
    return a * Math.PI / 180;
  }
  
  /**
   * 
   * @param a
   * @return angle in [0, 360)
   */
  public static final double toUnitAngle(double a) {
    return a % 360.0;
  }
  
  /**
   * 
   * @param a deg
   * @param b deg
   * @return difference between two angles in [-180, 180)
   */
  public static final double angleDiff(double a, double b) {
    double d = (b - a) % 360.0;
    if (d < -180) d += 360;
    else if (d >= 180) d -= 360;
    return d;
  }

  /**
   * 
   * @param a deg
   * @param b deg
   * @return
   */
  public static final double absAngleDiff(double a, double b) {
    return Math.abs(MathPP.angleDiff(a, b));
  }
}
