package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final ArmFeedforward feedforwardController =
      new ArmFeedforward(ArmConstants.GAINS.KS(), ArmConstants.GAINS.KG(), ArmConstants.GAINS.KV());

  // @RequiredArgsConstructor
  public enum ArmStates {
    DOWN_INTAKE(ArmConstants.BOT_ANGLE),
    MIDDLE_OUTTAKE(ArmConstants.MID_ANGLE),
    UP_INTAKE(ArmConstants.TOP_ANGLE),
    IDLE(null);

    Double value;

    ArmStates(Double value) {
      this.value = value;
    }

    public Double position() {
      return this.value;
    }
  }

  @AutoLogOutput(key = "Arm/Goal")
  private ArmStates goal = ArmStates.IDLE;

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    double ffVoltage =
        feedforwardController.calculate(
            inputs.positionDeg * Math.PI / 180.0, inputs.velocityDegPerSec * Math.PI / 180.0);
    // System.out.println(ffVoltage);

    switch (goal) {
      case DOWN_INTAKE -> io.runPosition(ArmStates.DOWN_INTAKE.position(), ffVoltage);

      case MIDDLE_OUTTAKE -> io.runPosition(ArmStates.MIDDLE_OUTTAKE.position(), ffVoltage);

      case UP_INTAKE -> io.runPosition(ArmStates.UP_INTAKE.position(), ffVoltage);

      default -> io.stop();
    }
  }

  public void setArmGoal(ArmStates desiredGoal) {
    goal = desiredGoal;
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionDeg - goal.position()) < ArmConstants.ARM_POSITION_TOLERANCE_DEG;
  }

  public void testPeriodic() {}
}
