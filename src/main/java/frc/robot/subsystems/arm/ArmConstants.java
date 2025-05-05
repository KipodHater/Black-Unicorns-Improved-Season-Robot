package frc.robot.subsystems.arm;

import frc.robot.Constants;


public class ArmConstants {
    public static final double ARM_MAX_VELOCITY = 800, ARM_MAX_ACCELARATION = 1000; // deg/s, deg/s2

    public static final Gains GAINS =
        switch(Constants.currentMode){
        case REAL -> new Gains(0.26, 0, 0.01, 0.01, 0.1, 0.0, 0.1);
        default -> new Gains(0.1, 0, 0, 0, 0, 0, 0);
    };

    public static final int ARM_CURRENT_LIMIT = 12;
    public static final double ARM_ENCODER_OFFSET = 55.0-16 - 40+74+6;
    public static final double ARM_POSITION_TOLERANCE_DEG = 1.0;
    public static final boolean ARM_INVERTED = false;
    public static final boolean ARM_BRAKE = true;
    
    public static final int FOLLOWER_ID = 9;
    public static final int MOTOR_ID = 8;

    public static final boolean MOTOR_INVERTED = false;

    public static final double MID_ANGLE = 44.5; //47.5
    public static final double TOP_ANGLE = 80.0;  //76
    public static final double BOT_ANGLE = 348.5; //355.5
    public static final double CLIMB_ANGLE = 345;

    public record Gains(double KP, double KI, double KD, double KS, double KV, double KA, double KG) {}
}
