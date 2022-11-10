package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private static final WPI_TalonFX intakeMotor = RobotMap.intakeMotor;


public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);

}

public void setIntakeSpeed(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
}

public void stop() {
    setIntakeSpeed(0);
}
}