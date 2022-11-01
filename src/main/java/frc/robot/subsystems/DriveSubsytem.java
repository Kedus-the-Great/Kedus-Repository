// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.RobotMap;

public class DriveSubsytem extends SubsystemBase {
  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDiveMotor;
private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;

private static final double IN_TO_M = .0254;
private static final int MOTOR_ENCODER_CODES_PER_REV =  2048;
private static final double DIAMETER_INCHES = 5.0;
private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M;
private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
private static final double GEAR_RATIO = 12.75;
private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
  /** Creates a new DriveSubsytem. */
  public DriveSubsytem() {
resetEncoders();
leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());
leftFrontMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_2_Feedback0, 1);
leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
leftFrontMotor.configVelocityMeasurmentPeriod(VelocityMeasPeriod.Period_10Ms);
leftFrontMotor.configVelocityMeasurmentWindow(16);//1,2,4,8,16,32,64(default)
leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
rightFrontMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_2_Feedback0, 1);
rightFrontMotor.configVelocityMeasurmentPeriod(VelocityMeasPeriod.Period_10Ms);
rightFrontMotor.configVelocityMeasurmentWindow(16);//1,2,4,8,16,32,64(default)
rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
leftBackMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_2_Feedback0, 1);
leftBackMotor.configVelocityMeasurmentPeriod(VelocityMeasPeriod.Period_10Ms);
leftBackMotor.configVelocityMeasurmentWindow(16);//1,2,4,8,16,32,64(default)
leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
rightBackMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_2_Feedback0, 1);
rightBackMotor.setStatusFramePeriod(StatusFrameEnchanced.Status_2_Feedback0, 1);
rightBackMotor.configVelocityMeasurmentWindow(16);//1,2,4,8,16,32,64(default)
rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Satus_12_Feedback1, 5, 10);
leftFrontMotor.setNeutral(NeutralMode.Coast);
rightFrontMotor.setNeutral(NeutralMode.Coast);
leftBackMotor.setNeutral(NeutralMode.Coast);
rightBackMotor.setNeutral(NeutralMode.Coast);
leftFrontMotor.setInverted(false);
rightFrontMotor.setInverted(true);
leftBackMotor.setInverted(false);
rightBackMotor.setInverted(true);
  }
public static void drive(double throttle, double rotate) {
  leftFrontMotor.set(throttle + rotate);
  rightFrontMotor.set(throttle - rotate);
  leftBackMotor.set(throttle + rotate);
  rightBackMotor.set(throttle - rotate);
}
public void resetEncoders(); {
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
  leftVelocityMPS = leftVelocityMPS * Meters_PER_TICKS;
  return (leftVelocityMPS);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
