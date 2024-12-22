package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.motor.SparkModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkModule intake = new SparkModule(14, false, false);
    private final double INTAKE_SPEED = 0.6;
    private final double RELEASE_SPEED = -0.6;

    public IntakeSubsystem() {
        super("Intake");
    }

    public void executeIntake() {
        this.intake.set(this.INTAKE_SPEED);
    }

    public void releaseIntake() {
        this.intake.set(this.RELEASE_SPEED);
    }

    public Command cmdExecute() {
        return Commands.runEnd(this::executeIntake, this::stopIntake, this);
    }
    
    public Command cmdRelease() {
        return Commands.runEnd(this::releaseIntake, this::stopIntake, this);
    }

    public void stopIntake() {
        this.intake.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Intake/Velocity", this.intake.getVelocity());
    }
}
