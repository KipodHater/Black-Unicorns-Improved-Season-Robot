package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
    // Add your code here
    private GripperIO io;
    protected final GripperIO.GripperIOInputs inputs;

    public Gripper(GripperIO io) {
        this.io = io;

        this.inputs = new GripperIO.GripperIOInputs();
    }

    public void setGripperMotorSpeed(double speed) {
        io.setSpeedRPM(speed);
    }

    public void setGripperMotorVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stopGripperMotor() {
        io.stop();
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public double getGripperMotorSpeed() {
        return inputs.gripperMotorRPM;
    }

    public double getGripperMotorVoltage() {
        return inputs.gripperMotorVoltage;
    }

    public double getGripperMotorTemp() {
        return inputs.gripperMotorTemp;
    }

    public void setPID(double KP, double KI, double KD) {
        io.setPID(KP, KI, KD);
    }

    @Override
    public void periodic() {
        
        updateInputs();

        SmartDashboard.putNumber("Gripper/GripperSpeed", inputs.gripperMotorRPM);

        SmartDashboard.putNumber("Gripper/GripperVoltage", inputs.gripperMotorVoltage);
        SmartDashboard.putNumber("Gripper/GripperTemp", inputs.gripperMotorTemp);
    }

    public void testPeriodic(){

    }

    // public void gripperAutonInit() {
    //     io.gripperAutonInit();
    // }

    

}