package frc.robot.subsystems.gripper;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.motorcontrol.spark.SparkMax;

public class GripperIOSparkMax implements GripperIO {
  private final SparkMax motor;
  private final BangBangController motorBangBangController;

  public private GripperIOSparkMax() {
    motor = new SparkMax(GripperConstants.K_SPARK_ID);
    motorBangBangController = new BangBangController();
  }
}
