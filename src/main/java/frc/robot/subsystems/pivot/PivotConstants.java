package frc.robot.subsystems.pivot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;

public class PivotConstants { // TODO: tune every single fucking thing here
  public static final double PIVOT_MAX_VELOCITY = 150, PIVOT_MAX_ACCELARATION = 550; // deg/s, deg/s2
  public static final Constraints PIVOT_CONSTRAINTS =
      new Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCELARATION);

  public static final Gains GAINS =
      switch (Constants.currentMode) {
          case REAL -> new Gains(0.065, 0, 0.0, 0.01, 0.01, 0.0, 0.04);
          default -> new Gains(0.1, 0, 0, 0, 0, 0, 0);
      };

  public static final double PIVOT_GEAR_RATIO = 14.0 / (36.0 * 60.0); // 100:1
  public static final double PIVOT_LENGTH_METERS = 1.0; // meters, change!
  public static final double PIVOT_MOI = 1.5; // kg*m^2
  public static final double MAX_ANGLE = 92.0; // degrees
  public static final double MIN_ANGLE = -30.0; // degrees

  public static final int PIVOT_CURRENT_LIMIT = 12;
  public static final double PIVOT_ENCODER_OFFSET = 207.2-1.5 -4.8  +118; // degs
  public static final double PIVOT_POSITION_TOLERANCE_DEG = 1.0;
  public static final IdleMode PIVOT_IDLE_MODE = IdleMode.kBrake;
  public static final double POSITION_CONVERSION_FACTOR = 360; // makes it degrees
  public static final double VELOCITY_CONVERSION_FACTOR = 60; // makes it rpm

  public static final int MOTOR_ID = 19;

  public static final boolean MOTOR_INVERTED = false;

  public static final double TOP_OUTTAKE_ANGLE = 175;
  public static final double TOP_ANGLE = 152.5; // 76
  public static final double MID_ANGLE = 92.0; // 47.5
  public static final double BOT_ANGLE = 117.5; // 355.5
  public static final double CLIMB_ANGLE = BOT_ANGLE; // TODO: what is this?

  public record Gains(
      double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
