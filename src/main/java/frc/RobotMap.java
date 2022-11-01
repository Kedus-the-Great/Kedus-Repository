package frc;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class RobotMap {
    public static final int LEFT_FRONT_DRIVE_PORT = 1;
    public static final int RIGHT_FRONT_DRIVE_PORT = 2;
    public static final int LEFT_BACK_DRIVE_PORT = 3;
    public static final int RIGHT_BACK_DIVE_PORT = 4;

    public static WPI_TalonFX leftFrontDiveMotor = new WPI_TalonFX(LEFT_FRONT_DRIVE_PORT);
    public static WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFX(RIGHT_FRONT_DRIVE_PORT);
    public static WPI_TalonFX leftBackDriveMotor = new WPI_TalonFX(LEFT_BACK_DRIVE_PORT);
    public static WPI_TalonFX rightBackDriveMotor = new WPI_TalonFX(RIGHT_BACK_DIVE_PORT);
}
