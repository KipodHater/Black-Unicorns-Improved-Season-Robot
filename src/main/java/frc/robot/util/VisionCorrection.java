package frc.robot.util;

public class VisionCorrection {

  /**
   * Corrects ty with enhanced edge correction Works well in center, stronger correction at edges
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

    // Base correction that works well in the middle
    double correctedTyRad = Math.atan(Math.tan(tyRad) / Math.cos(txRad));

    // Additional edge correction - increases non-linearly with tx
    // This accounts for lens distortion that's stronger at edges
    double txAbs = Math.abs(tx);

    // Edge correction factor - adjust these parameters
    double edgeThreshold = 15.0; // Degrees where edge correction starts to kick in
    double edgeStrength = 0.0015; // How strong the correction is (tune this)

    if (txAbs > edgeThreshold) {
      // Non-linear correction that increases at edges
      // Using (tx - threshold)² so it doesn't affect center much
      double edgeAmount = Math.pow(txAbs - edgeThreshold, 2);
      double edgeCorrection = edgeStrength * edgeAmount;

      // Apply correction (positive = makes ty more positive/up)
      // Flip sign if your distortion goes the other direction
      correctedTyRad += Math.toRadians(edgeCorrection);
    }

    return Math.toDegrees(correctedTyRad);
  }

  /**
   * Version with tunable edge parameters Use this to dial in the exact correction for your camera
   *
   * @param tx Horizontal angle to target (degrees)
   * @param ty Vertical angle to target (degrees)
   * @param cameraPitch Camera mount angle (degrees)
   * @param targetHeight Height of AprilTag center (meters)
   * @param cameraHeight Height of camera lens (meters)
   * @param edgeThreshold Degrees where edge correction starts (try 10-20)
   * @param edgeStrength How strong the edge correction is (try 0.001-0.003)
   * @param edgePower Exponent for edge falloff (2 = quadratic, 3 = cubic, etc.)
   * @return corrected ty
   */
  public static double getCorrectedTyTunable(
      double tx,
      double ty,
      double cameraPitch,
      double targetHeight,
      double cameraHeight,
      double edgeThreshold,
      double edgeStrength,
      double edgePower) {

    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);

    // Base correction
    double correctedTyRad = Math.atan(Math.tan(tyRad) / Math.cos(txRad));

    // Edge correction
    double txAbs = Math.abs(tx);

    if (txAbs > edgeThreshold) {
      double edgeAmount = Math.pow(txAbs - edgeThreshold, edgePower);
      double edgeCorrection = edgeStrength * edgeAmount;
      correctedTyRad += Math.toRadians(edgeCorrection);
    }

    return Math.toDegrees(correctedTyRad);
  }

  /**
   * Alternative: Smooth blending from center to edge Uses a smooth transition instead of a hard
   * threshold
   */
  public static double getCorrectedTySmooth(
      double tx, double ty, double cameraPitch, double targetHeight, double cameraHeight) {

    double txRad = Math.toRadians(tx);
    double tyRad = Math.toRadians(ty);

    // Base correction
    double correctedTyRad = Math.atan(Math.tan(tyRad) / Math.cos(txRad));

    // Smooth edge correction using a sigmoid-like function
    double txAbs = Math.abs(tx);

    // Blend factor: 0 at center, approaches 1 at edges
    // Adjust 20.0 to control where blending starts (lower = earlier)
    double blendFactor = 1.0 - Math.exp(-Math.pow(txAbs / 20.0, 2));

    // Additional correction at edges (tune this value)
    double maxEdgeCorrection = 0.05 * txAbs; // Increases linearly with tx

    // Apply blended correction
    correctedTyRad += Math.toRadians(maxEdgeCorrection * blendFactor);

    return Math.toDegrees(correctedTyRad);
  }

  /**
   * Helper method to test and compare corrections Use this to find the best parameters for your
   * camera
   */
  public static void printComparisonTable(
      double ty, double cameraPitch, double targetHeight, double cameraHeight) {
    System.out.println("TX\tRaw TY\tCorrected TY\tDifference");
    for (double tx = 0; tx <= 30; tx += 5) {
      double corrected = getCorrectedTy(tx, ty, cameraPitch, targetHeight, cameraHeight);
      System.out.printf("%.1f\t%.2f\t%.2f\t\t%.2f\n", tx, ty, corrected, corrected - ty);
    }
  }

  // Tuning guide:
  /*
  1. Start with getCorrectedTy() - it works well in middle

  2. If edges are still off, try getCorrectedTyTunable():
     - edgeThreshold: where does it start getting inaccurate? (typically 15-25°)
     - edgeStrength: how much correction needed? Start at 0.001, increase slowly
     - edgePower: 2 for smooth, 3 for more aggressive at far edges

  3. Test at known positions:
     double correctedTy = getCorrectedTyTunable(tx, ty, CAMERA_PITCH,
                                                 APRILTAG_HEIGHT, CAMERA_HEIGHT,
                                                 15.0,  // edgeThreshold
                                                 0.0018, // edgeStrength - TUNE THIS
                                                 2.0);   // edgePower

  4. If you want super smooth behavior, try getCorrectedTySmooth()
  */
}
