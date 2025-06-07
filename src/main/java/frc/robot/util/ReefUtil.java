package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ReefUtil {
  public enum BlueBranchPoses {
    A(new Pose2d(3.256, 4.195, new Rotation2d(0))),
    B(new Pose2d(3.262, 3.858, new Rotation2d(0))),
    C(new Pose2d(3.732, 3.034, new Rotation2d(Units.degreesToRadians(60)))),
    D(new Pose2d(4.019, 2.885, new Rotation2d(Units.degreesToRadians(60)))),
    E(new Pose2d(4.975, 2.896, new Rotation2d(Units.degreesToRadians(120)))),
    F(new Pose2d(5.263, 3.045, new Rotation2d(Units.degreesToRadians(120)))),
    G(new Pose2d(5.727, 3.858, new Rotation2d(Units.degreesToRadians(180)))),
    H(new Pose2d(5.711, 4.190, new Rotation2d(Units.degreesToRadians(180)))),
    I(new Pose2d(5.257, 4.997, new Rotation2d(Units.degreesToRadians(240)))),
    J(new Pose2d(4.970, 5.162, new Rotation2d(Units.degreesToRadians(240)))),
    K(new Pose2d(4.036, 5.135, new Rotation2d(Units.degreesToRadians(300)))),
    L(new Pose2d(3.748, 5.013, new Rotation2d(Units.degreesToRadians(300))));

    private Pose2d pose;

    private BlueBranchPoses(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }

    public Rotation2d geRotation() {
      return pose.getRotation();
    }
  }

  public enum RedBranchPoses {}
}
