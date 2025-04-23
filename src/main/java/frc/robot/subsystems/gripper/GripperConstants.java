package frc.robot.subsystems.gripper;

public class GripperConstants {
  public static final double GRIPPER_KP = 0.1;
  public static final double GRIPPER_KI = 0;
  public static final double GRIPPER_KD = 0;
  public static final double KMAX_ACCEL = 0.5;
  public static final double KMAX_SPEED = 1;

  public static final double GRIPPER_OUTTAKE_SPEED = -0.29;
  public static final double GRIPPER_OUTTAKEFAST_SPEED = -0.37;
  public static final double GRIPPER_INTAKE_SPEED = 0.85;

  public static final int K_SPARK_ID = 18;
  public static final int K_BEAMBREAK_ID = 1;
  public static final int K_CURRENT_LIMIT = 65; // amps

  public static final boolean K_INVERTED = true;
  public static final boolean K_BRAKE = true;
}
