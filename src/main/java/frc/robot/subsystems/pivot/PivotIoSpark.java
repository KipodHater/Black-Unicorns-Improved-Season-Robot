package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.arm.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class PivotIoSpark implements PivotIO {

  private final SparkMax motor;
  private final SparkMaxConfig config;

  private final AbsoluteEncoder pivotEncoder;

  private final ProfiledPIDController pivotPIDController;

  private final Debouncer motorDebouncer = new Debouncer(0.5);

  private boolean brakeEnabled = true;

  @AutoLogOutput(key = "Arm/Setpoint")
  private Double armSetpoint;

  public PivotIoSpark() throws ClassNotFoundException { // TODO: remove this exception
    motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    pivotEncoder = motor.getAbsoluteEncoder();

    config.inverted(false); // TODO: set any config values to constants

    config.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(1) // TODO: please tune :3
        .velocityConversionFactor(1) // TODO: please tune :3
        .uvwMeasurementPeriod(10);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    pivotPIDController =
        new ProfiledPIDController(1, 1, 1, new Constraints(0, 0)); // TODO: tune this shittt

    throw new ClassNotFoundException("dont use pivot yet you need to tune it first");
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(
        motor,
        pivotEncoder::getPosition,
        (position) -> inputs.positionDeg = position > 180 ? position % 360 - 360 : position % 360);
    ifOk(motor, pivotEncoder::getVelocity, (velocity) -> inputs.velocityDegPerSec = velocity);

    ifOk(motor, motor::getBusVoltage, (voltage) -> inputs.motorVoltage = voltage);
    ifOk(motor, motor::getMotorTemperature, (temp) -> inputs.motorTemp = temp);
    ifOk(motor, motor::getOutputCurrent, (current) -> inputs.motorCurrent = current);

    inputs.motorConnected = motorDebouncer.calculate(sparkStickyFault);

    sparkStickyFault = false;
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    runVoltage(0);
  }

  @Override
  public void runPosition(double position, double feedforward) {
    runVoltage(feedforward + pivotPIDController.calculate(pivotEncoder.getPosition(), position));
  }

  @Override
  public void setPID(double KP, double KI, double KD) {
    pivotPIDController.setPID(KP, KI, KD);
  }

  @Override
  public void setConstraints(Constraints constraints) {
    pivotPIDController.setConstraints(constraints);
  }

  @Override
  public void setBrakeMode(boolean isBrake) {
    if (isBrake == brakeEnabled) {
      return;
    }
    brakeEnabled = isBrake;
    if (isBrake) config.idleMode(IdleMode.kBrake);
    else config.idleMode(IdleMode.kCoast);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
