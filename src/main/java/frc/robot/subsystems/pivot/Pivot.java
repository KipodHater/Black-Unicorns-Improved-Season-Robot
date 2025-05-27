package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.arm.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(0, 0, 0); // TODO: can this be also armfeedforward?

  // @RequiredArgsConstructor
  public enum PivotStates {
    DOWN_INTAKE,
    MIDDLE_OUTTAKE,
    UP_INTAKE,
    IDLE
  }

  @AutoLogOutput(key = "Pivot/Goal")
  private PivotStates goal = PivotStates.IDLE;

  public Pivot(PivotIO io) throws ClassNotFoundException {
    this.io = io;

    throw new ClassNotFoundException("dont touch this yet make constants"); // TODO: listen to him
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * Math.PI / 180.0, inputs.velocityDegPerSec * Math.PI / 180.0);

    switch (goal) {
      case DOWN_INTAKE:
        io.runPosition(0, ffVoltage); // TODO: tune
        break;

      case MIDDLE_OUTTAKE:
        io.runPosition(0, ffVoltage); // TODO: this
        break;

      case UP_INTAKE:
        io.runPosition(0, ffVoltage); // TODO: shit
        break;

      default:
        io.stop();
    }
  }

  public void setArmGoal(PivotStates desiredGoal) {
    goal = desiredGoal;
  }

  public void testPeriodic() {}
}
