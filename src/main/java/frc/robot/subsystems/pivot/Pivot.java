package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(
          PivotConstants.GAINS.KS(), PivotConstants.GAINS.KG(), PivotConstants.GAINS.KV());

  // @RequiredArgsConstructor
  public enum PivotStates {
    DOWN_INTAKE(PivotConstants.BOT_ANGLE),
    MIDDLE_OUTTAKE(PivotConstants.MID_ANGLE),
    UP_INTAKE(PivotConstants.TOP_ANGLE),
    IDLE(null);

    Double value;

    PivotStates(Double value) {
      this.value = value;
    }

    public Double position() {
      return this.value;
    }
  }

  @AutoLogOutput(key = "Pivot/Goal")
  private PivotStates goal = PivotStates.IDLE;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * Math.PI / 180.0, inputs.velocityDegPerSec * Math.PI / 180.0);

    switch (goal) {
      case DOWN_INTAKE -> io.runPosition(PivotStates.DOWN_INTAKE.position(), ffVoltage);

      case MIDDLE_OUTTAKE -> io.runPosition(PivotStates.DOWN_INTAKE.position(), ffVoltage);

      case UP_INTAKE -> io.runPosition(PivotStates.UP_INTAKE.position(), ffVoltage);

      default -> io.stop();
    }
  }

  public void setPivotGoal(PivotStates desiredGoal) {
    goal = desiredGoal;
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionDeg - goal.position()) < PivotConstants.POSITION_TOLERANCE;
  }

  public void testPeriodic() {}
}
