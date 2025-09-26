// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.gripper.*;
import frc.robot.subsystems.gripper.Gripper.GripperStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSpark;
import frc.robot.subsystems.vision.*;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Arm arm;
  private final Drive drive;
  private final Gripper gripper;
  private final Leds leds;
  private final Pivot pivot;
  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null;

  private final SuperStructure structure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);
  private final GenericHID mainDriver = new GenericHID(0);

  private DoubleSupplier m_controllerLeftX;
  private DoubleSupplier m_controllerLeftY;
  private DoubleSupplier m_controllerRightX;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_controllerLeftX = () -> mainDriver.getRawAxis(3);
    m_controllerLeftY = () -> -controller.getRawAxis(2);
    m_controllerRightX = () -> -mainDriver.getRawAxis(0);

    switch (Constants.currentMode) {
      case REAL:
        arm = new Arm(new ArmIOSpark());
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {},
                m_controllerLeftX,
                m_controllerLeftY,
                m_controllerRightX);
        gripper = new Gripper(new GripperIOSpark());
        leds = new Leds();
        pivot = new Pivot(new PivotIOSpark());
        vision =
            new Vision(
                RobotState.getInstance()::addVisionObservation,
                new VisionIO[] {
                  /*
                  new VisionIOPhoton("camera0", VisionConstants.robotToCamera0),
                  new VisionIOPhoton("camera1", VisionConstants.robotToCamera1),
                  new VisionIOLimelight("limelight-tsachi", RobotState.getInstance()::getYaw)*/
                });
        break;

      case SIM:
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        
        arm = new Arm(new ArmIOSim());
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                (robotPose) -> driveSimulation.getSimulatedDriveTrainPose(),
                controller::getLeftX,
                controller::getLeftY,
                () -> controller.getRawAxis(4));
        gripper = new Gripper(new GripperIOSim(driveSimulation));
        leds = new Leds();
        pivot = new Pivot(new PivotIO() {});
        vision =
            new Vision(
                RobotState.getInstance()::addVisionObservation,
                new VisionIO[] {
                  new VisionIOPhotonSim(
                      "camera0",
                      VisionConstants.robotToCamera0,
                      driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonSim(
                      "camera1",
                      VisionConstants.robotToCamera1,
                      driveSimulation::getSimulatedDriveTrainPose)
                  // new VisionIOTest()
                });
        break;

      default:
        // Replayed robot, disable IO implementations
        arm = new Arm(new ArmIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {},
                controller::getLeftX,
                controller::getLeftY,
                () -> controller.getRawAxis(4));
        gripper = new Gripper(new GripperIO() {});
        leds = new Leds();
        pivot = new Pivot(new PivotIO() {});
        vision = new Vision(RobotState.getInstance()::addVisionObservation, new VisionIO[] {});
        break;
    }

    structure = new SuperStructure(arm, drive, gripper, leds, pivot, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    controller.a().onTrue(Commands.runOnce(structure::intakeButtonPress, structure));
    controller.b().onTrue(Commands.runOnce(structure::defaultButtonPress, structure));
    controller.rightBumper().onTrue(Commands.runOnce(structure::alignButtonPress, structure));
    controller.rightTrigger().onTrue(Commands.runOnce(structure::placeButtonPress, structure));

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> RobotState.getInstance().resetPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> RobotState.getInstance().resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    new Trigger(() -> mainDriver.getRawButton(6)).onTrue(Commands.runOnce(resetGyro).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    RobotState.getInstance().resetPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public void autonomousInit() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Reset the simulation to the initial pose
    driveSimulation.setSimulationWorldPose(drive.getPose());
    gripper.autonomousInit();
  }
}
