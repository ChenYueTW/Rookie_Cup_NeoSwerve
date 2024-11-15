package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.SparkModule;

public class IntakeSubsystem extends SubsystemBase implements IDashboardProvider {
    // TODO
    private final SparkModule intake = new SparkModule(0, false, false);
    private final double INTAKE_SPEED = 0.0;
    private final double RELEASE_SPEED = 0.0;
    private final double AMP_SPEED = 0.0;

    public IntakeSubsystem() {
        this.registerDashboard();
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

    public void stopIntake() {
        this.intake.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Intake Vel", this.intake.getVelocity());
    }
}
