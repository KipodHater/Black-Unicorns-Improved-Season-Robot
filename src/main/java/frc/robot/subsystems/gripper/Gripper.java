package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  // Add your code here
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  public Gripper(GripperIO io) {

    this.io = io;
  }

  public void setPID(double KP, double KI, double KD) {
    io.setPID(KP, KI, KD);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
  }

  public void testPeriodic() {}

  // public void gripperAutonInit() {
  //     io.gripperAutonInit();
  // }

}
