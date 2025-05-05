package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;
import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class GripperIOSim implements GripperIO {
    
    private FlywheelSim flywheeelSim;
    // private DCMotor gearBox;
    private PIDController flywheelPIDController;
    private SimpleMotorFeedforward ffController;
    private Double rpmVelocitySetpoint;

    private Double appliedVoltage;


    public GripperIOSim() {

        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), GRIPPER_MOMENT_OF_INNERTIA, gearRatio), DCMotor.getNEO(1));

        flywheelPIDController = new PIDController(GAINS.KP(), GAINS.KI(), GAINS.KD());
        ffController = new SimpleMotorFeedforward(GAINS.KS(), GAINS.KV(), GAINS.KA());   
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        flywheeelSim.update(CYCLE_TIME);

        inputs.gripperMotorRPM = flywheeelSim.getAngularVelocityRPM();
        inputs.gripperMotorVoltage = appliedVoltage;
        inputs.motorConnected = true;
    }

    @Override
    public void setSpeedRPM(double speed) {
        rpmVelocitySetpoint = speed;
        double ffVoltage = ffController.calculate(flywheeelSim.getAngularVelocityRPM());

        setVoltageOpenLoop(ffVoltage + flywheelPIDController.calculate(flywheeelSim.getAngularVelocityRPM()));
    }

    @Override
    public void setVoltageOpenLoop(double voltage) {
        rpmVelocitySetpoint = null;
        appliedVoltage = MathUtil.clamp(voltage, -12, 12);
        flywheeelSim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stop() {
        setVoltageOpenLoop(0);
    }

    public void setPID(double KP, double KI, double KD) {
        flywheelPIDController = new PIDController(KP, KI, KD);
    }

    public void setFF(double KS, double KV, double KA) {
        ffController = new SimpleMotorFeedforward(KS, KV, KA);
    }
}
