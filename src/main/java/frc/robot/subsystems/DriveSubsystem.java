// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
  private static final WPI_TalonFX leftFrontMotor= RobotMap.leftFrontDriveMotor;
  private static final WPI_TalonFX rightFrontMotor= RobotMap.rightFrontDriveMotor;
  private static final WPI_TalonFX leftBackMotor= RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor= RobotMap.rightBackDriveMotor;

  private static final double IN_TO_M = .0254;
  private static final int MOTOR_ENCODER_CODES_PER_REV =  2048;
  private static final double DIAMETER_INCHES = 5.0;
  private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M;
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double GEAR_RATIO = 12.75;
  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  /** Creates a new DriveSubsytem. */
  public DriveSubsystem() {
    resetEncoders();

    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

    leftFrontMotor.setInverted(true);
    rightFrontMotor.setInverted(false);
    leftBackMotor.setInverted(true); 
    rightBackMotor.setInverted(false);

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftFrontMotor.configNominalOutputForward(0, 10);
    leftFrontMotor.configNominalOutputReverse(0, 10);
    leftFrontMotor.configPeakOutputForward(1, 10);
    leftFrontMotor.configPeakOutputReverse(-1, 10);
    leftFrontMotor.configNeutralDeadband(0.001, 10);

    rightFrontMotor.configNominalOutputForward(0, 10);
    rightFrontMotor.configNominalOutputReverse(0, 10);
    rightFrontMotor.configPeakOutputForward(1, 10);
    rightFrontMotor.configPeakOutputReverse(-1, 10);
    rightFrontMotor.configNeutralDeadband(0.001, 10);

    leftBackMotor.configNominalOutputForward(0, 10);
    leftBackMotor.configNominalOutputReverse(0, 10);
    leftBackMotor.configPeakOutputForward(1, 10);
    leftBackMotor.configPeakOutputReverse(-1, 10);
    leftBackMotor.configNeutralDeadband(0.001, 10);

    rightBackMotor.configNominalOutputForward(0, 10);
    rightBackMotor.configNominalOutputReverse(0, 10);
    rightBackMotor.configPeakOutputForward(1, 10);
    rightBackMotor.configPeakOutputReverse(-1, 10);
    rightBackMotor.configNeutralDeadband(0.001, 10);

    // Sets how much error is allowed
    leftFrontMotor.configAllowableClosedloopError(0, 0, 10);
    leftBackMotor.configAllowableClosedloopError(0, 0, 10);
    rightFrontMotor.configAllowableClosedloopError(0, 0, 10);
    rightBackMotor.configAllowableClosedloopError(0, 0, 10);

    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
  }

  public static void drive(double throttle, double rotate) {
    leftFrontMotor.set(throttle + rotate);
    rightFrontMotor.set(throttle - rotate);
    leftBackMotor.set(throttle + rotate);
    rightBackMotor.set(throttle - rotate);
  }

  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }
  
  public double getRightBackEncoderPosition() {
    return rightBackMotor.getSelectedSensorPosition();
  }

  public double getLeftBackEncoderPosition() {
    return leftBackMotor.getSelectedSensorPosition();
  }

  public double distanceTravelledinTicks() {
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition()) / 2;
  }

  public double getLeftBackEncoderVelocityMetersPerSecond() {
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity()*10);
    leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
    return (leftVelocityMPS);
  }

  public double leftDistanceTravelledInMeters() {
    double left_dist = getLeftBackEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }
  
  public void stop() {
    drive(0,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}