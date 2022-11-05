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
 execute() {
     double throttle = driverController.getY();
    double rotate = driverController.getX();

    if ((throttle > 0 && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
        throttle = 0;
    } else {
        throttle = throttle;
    }

    if ((rotate > 0 && rotate < 0.25) || (rotate < 0 && rotate > -0.25)) {
        rotate = 0;
    } else {
    rotate = 2*rotate;
    }

    if (driverController.getTriggerAxis() > 0.25) {
        throttle = Math.signum(throttle) * 0.75;
    }
    else if (driverController.getAButton()) {
        throttle = (throttle*1.1);
    }
    else {
        throttle = (throttle*0.8);
    }
    driveSubsystem.drive(throttle, rotate);
 }





}