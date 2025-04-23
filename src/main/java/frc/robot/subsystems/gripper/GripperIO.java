package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;


public interface GripperIO {

    @AutoLog
    public static class GripperIOInputs {
        public boolean motorConnected = false;

        public double gripperMotorRPM = 0; //rots/minute
        public double gripperMotorVoltage = 0; //volts
        public double gripperMotorTemp = 0; // celsius
    }

    default public void updateInputs(GripperIOInputs inputs) {}
    
    default public void setSpeedRPM(double speed) {}

    default public void setVoltage(double voltage) {}

    default public void stop() {}

    default public void setPID(double KP, double KI, double KD) {}
    
    default public void setFF(double KS, double KV, double KA) {}
} 
    