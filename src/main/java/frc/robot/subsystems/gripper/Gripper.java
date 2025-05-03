package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class Gripper extends SubsystemBase {

  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  // @RequiredArgsConstructor
  public enum Goal{
    IDLE,
    OUTTAKE,
    OUTTAKE_STRONG,
    INTAKE,
    HOLD
  };

  @AutoLogOutput (key = "Gripper/Goal")
  private Goal goal = Goal.IDLE;

  public Gripper(GripperIO io) {
    this.io = io;
  }



  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);

    switch(goal){
      case IDLE: io.stop();
      case OUTTAKE: io.setVoltageOpenLoop(GRIPPER_OUTTAKE_SPEED);
      case OUTTAKE_STRONG: io.setVoltageOpenLoop(GRIPPER_OUTTAKEFAST_SPEED);
      case INTAKE: io.setVoltageOpenLoop(GRIPPER_INTAKE_SPEED);
      case HOLD: io.setVoltageOpenLoop(0.03);
    };
  }

  public void setGoalIdle(){
    goal = Goal.IDLE;
  }

  public void setGoalOuttake(){
    goal = Goal.OUTTAKE;
  }

  public void setGoalOuttakeFast(){
    goal = Goal.OUTTAKE_STRONG;
  }

  public void setGoalIntake(){
    goal = Goal.INTAKE;
  }

  public void setGoalHold(){
    goal = Goal.HOLD;
  }

  public void testPeriodic() {}
}
