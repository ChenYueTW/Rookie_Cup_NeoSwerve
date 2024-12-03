package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.motor.SparkModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // TODO
    private final SparkModule intake = new SparkModule(0, false, false);
    private final double INTAKE_SPEED = 0.0;
    private final double RELEASE_SPEED = 0.0;
    private final double AMP_SPEED = 0.0;

    public IntakeSubsystem() {
        super("Intake");
    }

    public void executeIntake() {
        this.intake.set(this.INTAKE_SPEED);
    }

    public void releaseIntake() {
        this.intake.set(this.RELEASE_SPEED);
    }

    public void releaseAmp() {
        this.intake.set(this.AMP_SPEED);
    }

    public Command cmdExecute() {
        return Commands.runEnd(this::executeIntake, this::stopIntake, this);
    }

    public void stopIntake() {
        this.intake.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Intake Vel", this.intake.getVelocity());
    }
}
