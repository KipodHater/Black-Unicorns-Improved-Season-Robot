// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveStates;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;

public class SuperStructure extends SubsystemBase {

  public  enum SuperStructureStates {
    TRAVEL,
    INTAKE_CORAL_FLOOR,
    ALIGN_L1,
    PLACE_L1, // can possibly add modes for scoring different coral heights
    CLIMB
  }

  @AutoLogOutput(key = "SuperStructure/currentState")
  private SuperStructureStates currentState = SuperStructureStates.TRAVEL;

  @AutoLogOutput(key = "SuperStructure/wantedState")
  private SuperStructureStates wantedState = SuperStructureStates.TRAVEL;

  private SuperStructureStates previousState = SuperStructureStates.TRAVEL;

  public Command currentCommand = null;

  private final Arm arm;
  private final Drive drive;
  private final Gripper gripper;
  private final Pivot pivot;
  private final Vision vision;

  public SuperStructure(Arm arm, Drive drive, Gripper gripper, Pivot pivot, Vision vision) {
    this.arm = arm;
    this.drive = drive;
    this.gripper = gripper;
    this.pivot = pivot;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    previousState = currentState;
    if(wantedState != currentState) currentState = handleStateTransition(wantedState);
    wantedState = currentState;
    stateMachine();
  }

  private SuperStructureStates handleStateTransition(SuperStructureStates wantedState) {
    return switch (wantedState) {
      case TRAVEL -> SuperStructureStates.TRAVEL;
      case INTAKE_CORAL_FLOOR -> SuperStructureStates.INTAKE_CORAL_FLOOR;
      case ALIGN_L1 -> SuperStructureStates.ALIGN_L1;
      case PLACE_L1 -> currentState == SuperStructureStates.ALIGN_L1 ? SuperStructureStates.PLACE_L1 : currentState;
      case CLIMB -> SuperStructureStates.CLIMB;
    };
  }

  private void stateMachine() {
    switch (currentState) {
      case TRAVEL -> {
        if(previousState != SuperStructureStates.TRAVEL) {
          if(currentCommand != null) currentCommand.cancel();
          currentCommand = null;
        }
        arm.setArmGoal(Arm.ArmStates.MIDDLE_OUTTAKE);
        pivot.setPivotGoal(Pivot.PivotStates.MIDDLE_OUTTAKE);
        gripper.setGripperGoal(Gripper.GripperStates.HOLD);
        drive.setState(DriveStates.FIELD_DRIVE);
      }
      case INTAKE_CORAL_FLOOR -> {
        arm.setArmGoal(Arm.ArmStates.DOWN_INTAKE);
        pivot.setPivotGoal(Pivot.PivotStates.DOWN_INTAKE);
        gripper.setGripperGoal(Gripper.GripperStates.INTAKE);
        drive.setState(DriveStates.FIELD_DRIVE); // can possibly add assisted drive
      }
      case ALIGN_L1 -> {
        
      }
    }
  }

  public void setWantedState(SuperStructureStates wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(SuperStructureStates wantedState) {
    return runOnce(() -> setWantedState(wantedState));
  }

  public SuperStructureStates getCurrentState() {
    return currentState;
  }
}
