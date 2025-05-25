package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotState;

public class VisionIOLimelight implements VisionIO{

    private final Supplier<Rotation2d> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;

    private final NetworkTable table;

    
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    private final ;

    private boolean isHighRes = true;

    public VisionIOLimelight (String name, Supplier<Rotation2d> rotationSupplier){
        // this.name = name;

        this.rotationSupplier = rotationSupplier;

        table = NetworkTableInstance.getDefault().getTable(name);

        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();


    }

    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

        inputs.latestTargetObservation = 
            new TargetObservation(Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));


    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    
    }

    private void setHighResPipeline(boolean highRes) {
        if(highRes == isHighRes) return;
        isHighRes = highRes;
        table.getEntry("pipeline").setNumber(highRes ? VisionConstants.highResPipeline : VisionConstants.lowResPipeline);
    }

    private void cropStream(double minX, double maxX, double minY, double maxY){ // all values are (-1.0, 1.0)
        double[] cropArray = {minX, maxX, minY, maxY};
        table.getEntry("crop").setDoubleArray(cropArray);
    }
}
