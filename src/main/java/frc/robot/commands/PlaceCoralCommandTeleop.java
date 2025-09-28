package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.ledsStates;
import frc.robot.subsystems.pivot.Pivot;

public class PlaceCoralCommandTeleop extends SequentialCommandGroup {

  public PlaceCoralCommandTeleop(Arm arm, Drive drive, Gripper gripper, Leds leds, Pivot pivot) {
    addRequirements(arm, drive, gripper, leds, pivot);

    addCommands(
        Commands.parallel(
            Commands.runOnce(() -> leds.setState(ledsStates.FINISH_SCORE)),
            Commands.runOnce(() -> gripper.setGripperGoal(Gripper.GripperStates.OUTTAKE_STRONG))));
    addCommands(Commands.waitSeconds(0.3));
    addCommands(
        Commands.runOnce(
            () ->
                drive.setStateSlowlyForward(
                    RobotState.getInstance().getCoralScoringInfo().backside())));

    addCommands(Commands.waitSeconds(0.3));
  }
}
