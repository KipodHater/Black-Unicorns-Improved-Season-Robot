package frc.robot.subsystems.objectVision;

import static frc.robot.subsystems.objectVision.ObjectVisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class ObjectVision {

  // private final ObjectVisionConsumer consumer;

  private final ObjectVisionIO[] io;
  private final ObjectVisionIOInputsAutoLogged[] inputs;
  private final Function<Double, Pose2d> poseFunction;
  private final Alert[] alerts;

  public ObjectVision(Function<Double, Pose2d> poseFunction, ObjectVisionIO... io) {

    this.io = io;
    this.inputs = new ObjectVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) inputs[i] = new ObjectVisionIOInputsAutoLogged();

    this.poseFunction = poseFunction;
    this.alerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++)
      alerts[i] =
          new Alert(
              "Object detection camera: " + io[i].getName() + " is disconnected.",
              Alert.AlertType.kWarning);
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("ObjectVision/Camera " + io[i].getName(), inputs[i]);
      alerts[i].set(!inputs[i].connected);
    }

    List<Translation2d> targetTranslations = new LinkedList<>();
    for (int i = 0; i < io.length; i++) {
      Pose2d fieldToRobot = poseFunction.apply(inputs[i].timestamp);
      Transform3d robotToCamera = robotToCameras[i];
      Rotation2d cameraPitch = new Rotation2d(robotToCameras[i].getRotation().getX());
      Rotation2d cameraYaw = new Rotation2d(robotToCameras[i].getRotation().getZ());

      for (var target : inputs[i].targets) {

        Rotation2d yaw =
            fieldToRobot
                .getRotation()
                .plus(projectToGround(cameraPitch, new Rotation2d(target.tx())))
                .plus(cameraYaw);
        double distance =
            robotToCamera.getY() * (cameraPitch.plus(new Rotation2d(target.ty()))).getCos();

        Translation2d robotToTarget =
            new Translation2d(
                fieldToRobot.getX() + distance * yaw.getCos(),
                fieldToRobot.getY() + distance * yaw.getSin());

        Translation2d targetTranslation = fieldToRobot.getTranslation().plus(robotToTarget);
        // TODO: add here clamping target values into field
        targetTranslations.add(targetTranslation);
      }
    }

    Logger.recordOutput(
        "ObjectVision/TargetTranslations",
        targetTranslations.toArray(new Translation2d[targetTranslations.size()]));
  }

  private Rotation2d projectToGround(Rotation2d cameraPitch, Rotation2d tx) {
    return new Rotation2d(Math.atan(1.0 * cameraPitch.getTan() / tx.getCos()));
  }

  // @FunctionalInterface
  // public static interface ObjectVisionConsumer {
  //     public void accept(
  //         Transform2d robotToTarget,
  //         double timestampSeconds);
  // }

}
