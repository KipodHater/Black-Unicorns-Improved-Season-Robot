package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {

  private final ProfiledPIDController pivotPIDController;
  private final SingleJointedArmSim pivotSim;

  private double appliedVoltage = 0;

  public PivotIOSim() {
    System.out.println("Pivot Sim Constructor");
    pivotPIDController =
        new ProfiledPIDController(GAINS.KP(), GAINS.KI(), GAINS.KD(), PIVOT_CONSTRAINTS);
    pivotSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), PIVOT_MOI, 100.0),
            DCMotor.getNEO(2),
            PIVOT_GEAR_RATIO,
            PIVOT_LENGTH_METERS,
            MIN_ANGLE * Math.PI / 180.0,
            MAX_ANGLE * Math.PI / 180.0,
            true,
            60 * Math.PI / 180.0,
            0.05,
            0.05);
    System.out.println("Arm Sim Initialized");
  }

  public void updateInputs(PivotIOInputs inputs) {
    pivotSim.update(Constants.CYCLE_TIME);
    inputs.positionDeg = pivotSim.getAngleRads() * 180 / Math.PI;
    inputs.velocityDegPerSec = pivotSim.getVelocityRadPerSec() * 180 / Math.PI;
    inputs.motorVoltage = appliedVoltage;
    inputs.motorTemp = 0;
    inputs.motorConnected = true;
  }

  public void runVoltage(double voltage) {
    appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    pivotSim.setInputVoltage(appliedVoltage);
  }

  public void stop() {
    runVoltage(0);
  }

  public void runPosition(double position, double feedforward) {
    double output = pivotPIDController.calculate(pivotSim.getAngleRads() * 180 / Math.PI, position);
    runVoltage(output + feedforward);
  }

  public void setPID(double KP, double KI, double KD) {
    pivotPIDController.setPID(KP, KI, KD);
  }

  public void setConstraints(Constraints constraints) {
    pivotPIDController.setConstraints(constraints);
  }

  public void setBrakeMode(boolean isBrake) {}
}
