package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm {

  private final ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // @RequiredArgsConstructor
  public enum ArmStates {
    DOWN_INTAKE,
    MIDDLE_OUTTAKE,
    UP_INTAKE,
    IDLE
  }

  @AutoLogOutput (key = "Arm/Goal")
  private ArmStates goal;

  public Arm(ArmIO io){
    this.io = io;
  }

  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    switch(goal){
        case DOWN_INTAKE:
            io.runPosition(0, 0);
            break;

        case MIDDLE_OUTTAKE:
            io.runPosition(0, 0);
            break;

        case UP_INTAKE:
            io.runPosition(0, 0);
            break;
        
        default:
            io.stop();
    }
  }

  public void setArmGoal(ArmStates desiredGoal){
    goal = desiredGoal;
  }

  public void testPeriodic() {}
}
