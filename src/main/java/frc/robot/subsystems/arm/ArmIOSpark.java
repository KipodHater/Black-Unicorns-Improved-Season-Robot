package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;

public class ArmIOSpark implements ArmIO{
    
    private final SparkMax motor;
    private final SparkMax follower;
    private final SparkMaxConfig config;
    private final SparkMaxConfig followerConfig;

    private final AbsoluteEncoder armEncoder;

    private ProfiledPIDController armPidController;

    @AutoLogOutput (key = "Arm/Setpoint")
    private Double armSetpoint;

    public ArmIOSpark() {
        motor = new SparkMax(ARM_MOTOR_ID);
        follower = new SparkMax(ARM_FOLLOWER_ID);

        config = new SparkMaxConfig();
        followerConfig = new SparkMaxConfig();

        armEncoder = motor.getAbsoluteEncoder();

        config.inverted(ARM_MOTOR_INVERTED);
        followerConfig.inverted(ARM_FOLLOWER_INVERTED);

        tryUntilOk(motor, 5, () -> motor.configure(config));
        tryUntilOk(follower, 5, () -> follower.configure(followerConfig));
    }
}
