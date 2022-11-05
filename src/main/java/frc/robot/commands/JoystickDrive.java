package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final static XboxController driverController = RobotContainer.driverController;

    public JoystickDrive(DriveSubsystem drivetrain) {
        driveSubsystem = drivetrain;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        double throttle = driverController.getLeftY();
        double rotate = driverController.getRightX();

        if ((throttle > 0 && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
            throttle = 0;
        } 

        if ((rotate > 0 && rotate < 0.25) || (rotate < 0 && rotate > -0.25)) {
            rotate = 0;
        } 

        driveSubsystem.drive(throttle*0.35, -rotate*0.35);
    }
}