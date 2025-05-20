// package frc.robot.subsystems.vision;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotController;
// import frc.robot.RobotState;

// public class VisionIOLimelight implements VisionIO{

//     private final String name;

//     private final Supplier<Rotation2d> rotationSupplier;
//     private final DoubleSupplier velocitySupplier;

//     private final NetworkTable table;

//     private final DoubleSubscriber latencySubscriber;
//     private final DoubleSubscriber txSubscriber;
//     private final DoubleSubscriber tySubscriber;

//     private final 

//     public VisionIOLimelight (String name){
//         this.name = name;

//         this.rotationSupplier = RobotState.getInstance() :: getYaw;
//         this.velocitySupplier = RobotState.getInstance() :: getVelocity;

//         table = NetworkTableInstance.getDefault().getTable(name);

//         latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
//         txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
//         tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

//         megat

//     }

//     public void updateInputs(VisionIOInputs inputs) {
//         inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

//         inputs.latestTargetObservation = 

//         inputs.poseObservations = new PoseObservation[0];

//         inputs.tagIds = table.getEntry("tag_ids");
//     }
// }
