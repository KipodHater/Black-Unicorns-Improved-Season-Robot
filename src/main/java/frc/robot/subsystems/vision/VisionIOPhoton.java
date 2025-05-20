package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;

public class VisionIOPhoton implements VisionIO {
    
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;

    public VisionIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        Set<Short> tagIds = new HashSet<>();
        var results = camera.getAllUnreadResults();
        for(var result : results) {
            if (result.hasTargets() && RobotController.getFPGATime() * 1e-6 - result.getTimestampSeconds() < 0.025) {
                inputs.latestTargetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            if(result.multitagResult.isPresent()){
                var multitagResult = result.multitagResult.get();

                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = robotToCamera.inverse().plus(fieldToCamera);
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                  }

                inputs.poseObservations = new PoseObservation[] {
                    new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / multitagResult.fiducialIDsUsed.size(),
                        )//add here a function that can calculate the FOM
                };
                
            }
        }
    }
}
