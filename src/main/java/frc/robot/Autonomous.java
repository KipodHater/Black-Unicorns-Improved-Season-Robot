package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

public class Autonomous {
  private final Drive drive;

  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static RobotConfig PP_CONFIG;
  private final PathConstraints constraints;

  private enum SelectedLoader {
    Left(new Pose2d(0.94, 0.87, new Rotation2d(-2.21))),
    Right(new Pose2d(1.05, 7.22, new Rotation2d(2.21))),
    CONTROL(new Pose2d(300.35, 400.18, new Rotation2d(Units.degreesToRadians(0))));

    private Pose2d pose;

    private SelectedLoader(Pose2d pose) {
      this.pose = pose;
    }
  }

  private enum ReefBranch {
    A(new Pose2d(3.35, 4.18, new Rotation2d(Units.degreesToRadians(0))), false),
    B(new Pose2d(3.35, 3.85, new Rotation2d(Units.degreesToRadians(0))), false),
    C(new Pose2d(3.77, 3.1, new Rotation2d(Units.degreesToRadians(60))), false),
    D(new Pose2d(4.05, 2.98, new Rotation2d(Units.degreesToRadians(60))), false),
    E(new Pose2d(4.91, 2.97, new Rotation2d(Units.degreesToRadians(120))), false),
    F(new Pose2d(5.18, 3.12, new Rotation2d(Units.degreesToRadians(120))), false),
    G(new Pose2d(5.61, 3.84, new Rotation2d(Units.degreesToRadians(180))), false),
    H(new Pose2d(5.61, 4.18, new Rotation2d(Units.degreesToRadians(180))), false),
    I(new Pose2d(5.20, 4.92, new Rotation2d(Units.degreesToRadians(240))), false),
    J(new Pose2d(4.92, 5.07, new Rotation2d(Units.degreesToRadians(240))), false),
    K(new Pose2d(4.07, 5.07, new Rotation2d(Units.degreesToRadians(300))), false),
    L(new Pose2d(3.76, 4.92, new Rotation2d(Units.degreesToRadians(300))), false),
    CONTROL(new Pose2d(300.35, 400.18, new Rotation2d(Units.degreesToRadians(0))), false);

    private Pose2d pose;

    private ReefBranch(Pose2d pose, boolean full) {
      this.pose = pose;
    }
  }

  public Autonomous(Drive drive) {

    this.drive = drive;

    PP_CONFIG =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getFalcon500(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            Drive.getModuleTranslations());

    AutoBuilder.configure(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        (speeds, feedforward) -> {
          drive.runVelocity(speeds);
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());

    constraints =
        new PathConstraints(
            MetersPerSecond.of(4),
            MetersPerSecondPerSecond.of(6),
            RadiansPerSecond.of(4),
            RadiansPerSecondPerSecond.of(6));
  }

  public double getDistance(Pose2d pose1, Pose2d pose2d) {
    return pose1.getTranslation().getDistance(pose2d.getTranslation());
  }

  public ReefBranch getClosestBranch(List<ReefBranch> fullBranchs) {
    ReefBranch curClosest = ReefBranch.CONTROL; // Default to CONTROL if no branches are defined
    Supplier<Pose2d> curPose = drive::getPose;
    for (ReefBranch branch : ReefBranch.values()) {
      if (fullBranchs.contains(branch)) {
        continue; // Skip branches that are full
      }
      if (branch == ReefBranch.CONTROL) {
        continue; // Skip the control branch
      }
      if (getDistance(curPose.get(), branch.pose) < getDistance(curPose.get(), curClosest.pose)) {
        curClosest = branch;
      }
    }

    fullBranchs.add(curClosest);

    return curClosest;
  }

  public SelectedLoader getClosestLoader() {
    SelectedLoader curClosest = null; // Start with no closest loader
    Supplier<Pose2d> curPose = drive::getPose;

    for (SelectedLoader loader : SelectedLoader.values()) {
      if (loader == SelectedLoader.CONTROL) {
        continue; // Skip the control loader
      }
      if (curClosest == null
          || getDistance(curPose.get(), loader.pose)
              < getDistance(curPose.get(), curClosest.pose)) {
        curClosest = loader;
      }
    }

    // If no valid loader is found, default to CONTROL
    return curClosest != null ? curClosest : SelectedLoader.CONTROL;
  }

  public Command loaderBranchLoop(int times) {
    List<ReefBranch> fullBranches = new ArrayList<>();
    Command auto;

    AtomicInteger count = new AtomicInteger(0);

    auto =
        new RepeatCommand(
                new SequentialCommandGroup(
                    Commands.defer(
                        () -> AutoBuilder.pathfindToPose(getClosestLoader().pose, constraints),
                        Set.of(drive)),
                    Commands.defer(
                        () ->
                            AutoBuilder.pathfindToPose(
                                getClosestBranch(fullBranches).pose, constraints),
                        Set.of(drive)),
                    Commands.runOnce(() -> count.incrementAndGet())))
            .withDeadline(Commands.waitUntil(() -> count.get() >= times));

    return auto;
  }

  public Command getAuto(boolean leftFeeder) {
    Command auto;
    Command subAuto;

    List<ReefBranch> fullBranches = new ArrayList<>();

    subAuto =
        Commands.sequence(
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(getClosestBranch(fullBranches).pose, constraints),
                Set.of(drive)),
            new SelectCommand<>(
                Map.of(
                    "L",
                    AutoBuilder.pathfindToPose(SelectedLoader.Left.pose, constraints),
                    "R",
                    AutoBuilder.pathfindToPose(SelectedLoader.Right.pose, constraints)),
                () -> leftFeeder ? "L" : "R" // Select left or right loader based on first char
                ),
            //

            loaderBranchLoop(5),
            Commands.none());

    auto = Commands.defer(() -> subAuto, Set.of(drive));
    auto.cancel();
    subAuto.cancel();
    return auto;
  }
}
