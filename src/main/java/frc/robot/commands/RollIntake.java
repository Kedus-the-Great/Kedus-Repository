// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class RollIntake extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
    private final XboxController driverController = RobotContainer.driverController;
  /** Creates a new RollIntake. */
  public RollIntake(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;
    addRequirements(intakeSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driverController.getBButton()) {
      intakeSubsystem.setIntakeSpeed(0.5);
    } else if (driverController.getAButton()) {
      intakeSubsystem.setIntakeSpeed(-0.5);
    } else {
      intakeSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
