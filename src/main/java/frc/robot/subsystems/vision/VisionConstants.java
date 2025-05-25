package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final short lowResPipeline = 1;
  public static final short highResPipeline = 0;

  public static final double lowResTriggerDistance = 3.0; // meters
  public static final double highResTriggerDistance = 3.5; // meters
}
