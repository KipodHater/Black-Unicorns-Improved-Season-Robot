package frc.robot.subsystems.objectVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ObjectVisionConstants {
  public static Transform3d[] robotToCameras =
      new Transform3d[] {
        new Transform3d(0.25, 0.4, 0, new Rotation3d(0, -0.5, 0)) // TODO: change to actual value
      };
}
