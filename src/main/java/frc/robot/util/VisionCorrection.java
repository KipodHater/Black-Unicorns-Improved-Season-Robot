package frc.robot.util;

public class VisionCorrection {

  /**
   * Corrects ty for large tx angles by accounting for perspective distortion
   *
   * @param tx Horizontal angle to target (degrees)
   * @param ty Vertical angle to target (degrees) - raw from PhotonVision
   * @param cameraPitch Camera mount angle (degrees, + = angled up)
   * @param targetHeight Height of AprilTag center (meters)
   * @param cameraHeight Height of camera lens (meters)
   * @return corrected_ty More accurate vertical angle (degrees)
   */
  public static double getCorrectedTy(
      double tx, double ty, double cameraPitch, double targetHeight, double cameraHeight) {

    // Convert to radians
    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);
    double pitchRad = Math.toRadians(cameraPitch);

    // The key insight: when looking at large tx, the effective vertical angle
    // is compressed due to the 3D rotation of the view plane

    // Correct ty by accounting for the horizontal rotation
    // As tx increases, the vertical component appears smaller
    double correctedTyRad = Math.atan(Math.tan(tyRad) / Math.cos(txRad));

    double correctedTy = Math.toDegrees(correctedTyRad);

    return correctedTy;
  }

  /**
   * Alternative method: Empirical polynomial correction Tune the coefficients based on real
   * measurements
   *
   * @param tx Horizontal angle to target (degrees)
   * @param ty Vertical angle to target (degrees)
   * @param a Linear coefficient (try 0.02-0.05)
   * @param b Quadratic coefficient (try 0.0005-0.001)
   * @return corrected ty with empirical adjustment
   */
  public static double getCorrectedTyEmpirical(double tx, double ty, double a, double b) {
    // Empirical correction: ty_corrected = ty + a*tx + b*tx²
    // The sign depends on your camera distortion pattern
    double txAbs = Math.abs(tx);
    double correction = a * txAbs + b * txAbs * txAbs;

    // Add correction (might need to be subtracted depending on your distortion)
    return ty + correction;
  }

  /**
   * More sophisticated 3D transformation approach Projects the target position through proper 3D
   * rotation
   */
  public static double getCorrectedTy3D(double tx, double ty, double cameraPitch) {
    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);
    double pitchRad = Math.toRadians(cameraPitch);

    // Create unit vector pointing at target in camera frame
    // Camera frame: X = right, Y = down, Z = forward
    double x = Math.tan(txRad);
    double y = Math.tan(tyRad);
    double z = 1.0;

    // Normalize
    double magnitude = Math.sqrt(x * x + y * y + z * z);
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;

    // Rotate by camera pitch to get robot frame
    // Robot frame: Y is still vertical
    double yRobot = y * Math.cos(pitchRad) + z * Math.sin(pitchRad);
    double zRobot = -y * Math.sin(pitchRad) + z * Math.cos(pitchRad);

    // Calculate the effective vertical angle in robot frame
    double correctedTyRad = Math.atan2(yRobot, Math.sqrt(x * x + zRobot * zRobot));

    return Math.toDegrees(correctedTyRad) - cameraPitch;
  }

  /** Returns a confidence value based on tx angle */
  public static double getConfidence(double tx) {
    double txRad = Math.toRadians(tx);
    return Math.pow(Math.cos(txRad), 2);
  }

  // Example usage - try all three methods and see which works best:
  /*
  double tx = target.getYaw();
  double ty = target.getPitch();

  // Method 1: Simple geometric correction
  double correctedTy1 = VisionCorrection.getCorrectedTy(tx, ty, CAMERA_PITCH,
                                                        APRILTAG_HEIGHT, CAMERA_HEIGHT);

  // Method 2: Empirical (tune a and b based on your measurements)
  double correctedTy2 = VisionCorrection.getCorrectedTyEmpirical(tx, ty, 0.03, 0.0008);

  // Method 3: Full 3D transformation
  double correctedTy3 = VisionCorrection.getCorrectedTy3D(tx, ty, CAMERA_PITCH);

  // Test which one gives you better results!
  */
}
