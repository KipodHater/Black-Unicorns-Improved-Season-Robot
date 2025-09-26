// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.Gripper.GripperStates;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;

public class AlignReefCommand extends SequentialCommandGroup {

  public AlignReefCommand(Drive drive, Arm arm, Gripper gripper, Leds leds, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    RobotState.getInstance().setUpScoringTargetCoral();
    addRequirements(drive, arm, pivot, gripper);
    addCommands(
            Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(() -> leds.setState(Leds.ledsStates.PURPLE), leds),
                    SimpleCommands.moveToScoreCommand(
                        arm,
                        pivot),
                    Commands.runOnce(() -> gripper.setGripperGoal(GripperStates.HOLD)),
                    SimpleCommands.driveAutoAlignTolerance(
                        drive,
                        () -> RobotState.getInstance().getCoralScoringInfo().scorePose(),
                        0.05,
                        3)),
                Commands.race(
                    SimpleCommands.nonStopAutoAlignCommand(
                        drive, () -> RobotState.getInstance().getCoralScoringInfo().alignPose()),
                    // Commands.waitSeconds(0.3)),
                Commands.parallel(
                    SimpleCommands.nonStopAutoAlignCommand(
                        drive, () -> RobotState.getInstance().getCoralScoringInfo().scorePose())),
                    SimpleCommands.blinkLedsOnAlignCondition(leds, () -> drive.isAtAlignSetpoint(0.03, 2)))));
  }
}
