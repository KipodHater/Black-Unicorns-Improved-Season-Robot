package frc.robot.subsystems.vision;

public class Vision {

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO... io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }
}
