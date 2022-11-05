// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
  private static final WPI_TalonFX leftFrontMotor= RobotMap.leftFrontDrivePort;
  private static final WPI_TalonFX rightFrontMotor= RobotMap.rightFrontDrivePort;
  private static final WPI_TalonFX leftBackMotor= RobotMap.leftBackDrivePort;
  private static final WPI_TalonFX rightBackMotor= RobotMap.rightBackDrivePort;

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

leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);

leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
//1,2,4,8,16,32,64(default)



leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
leftBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
rightFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
rightBackMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

leftFrontMotor.configVelocityMeasurementWindow(16);
leftBackMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
rightFrontMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
rightBackMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)

leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

leftFrontMotor.setNeutralMode(NeutralMode.Coast);
rightFrontMotor.setNeutralMode(NeutralMode.Coast);
leftBackMotor.setNeutralMode(NeutralMode.Coast);
rightBackMotor.setNeutralMode(NeutralMode.Coast);

leftFrontMotor.setInverted(false);
rightFrontMotor.setInverted(true);
leftBackMotor.setInverted(false);
rightBackMotor.setInverted(true);

frontLeftMotor.set(ControlMode.Follower, backLeftMotor.getDeviceID());
frontRightMotor.set(ControlMode.Follower, backRightMotor.getDeviceID());

leftMotor.setNeutralMode(NeutralMode.Coast);
rightMotor.setNeutralMode(NeutralMode.Coast);

leftMotorFront.configNominalOutputForward(0,10);
leftMotorFront.configNominalOutputReverse(0,10);
leftMotorFront.configPeakOutputForward(1,10);
leftMotorFront.configPeakOutputReverse(-1,10);

rightMotorFront.configNominalOutputForward(0,10);
rightMotorFront.configNominalOutputReverse(0,10);
rightMotorFront.configPeakOutputForward(1,10);
rightMotorFront.configPeakOutputReverse(-1,10);

leftMotorFront.configNeutralDeadband(0.001, 10);
leftMotorBack.configNeutralDeadband(0.001, 10);
rightMotorFront.configNeutralDeadband(0.001, 10);
rightMotorBack.configNeutralDeadband(0.001, 10);

frontLeftMotor.setSensorPhase(true);
frontRightMotor.setSensorPhase(false);
backLeftMotor.setSensorPhase(true);
backRightMotor.setSensorPhase(false);

frontLeftMotor.setInverted(true);
frontRightMotor.setInverted(false);
backLeftMotor.setInverted(true);
backRightMotor.setInverted(false);
}
public static void drive(double throttle, double rotate) {
  leftFrontMotor.set(throttle + rotate);
  rightFrontMotor.set(throttle - rotate);
  leftBackMotor.set(throttle + rotate);
  rightBackMotor.set(throttle - rotate);
}
public void resetEncoders() {
  backLeftMotor.setSelectedSensorPosition(0);
  backRightMotor.setSelectedSensorPosition(0);
  frontLeftMotor.setSelectedSensorPosition(0);
  frontRightMotor.setSelectedSensorPosition(0);
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
