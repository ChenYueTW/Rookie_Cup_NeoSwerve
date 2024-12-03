package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class IntakeLifterSubsystem extends SubsystemBase {
    private final TalonModule lifter = new TalonModule(13, false, true);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController lifterPid = new PIDController(0, 0, 0, 0.01); // TODO
    private final double AMP_DEGREE = 0.0; // TODO
    private final double MAX_DEGREE = 0.0; // TODO
    private final double MIN_DEGREE = 0.0; // TODO

    public IntakeLifterSubsystem() {
        super("Intake Liefter");
        this.lifter.setDegreesRange(this.MAX_DEGREE, this.MIN_DEGREE);
    }

    public void executeLifter(double speed) {
        this.lifter.set(speed);
    }

    public Command isDownDone() {
        return new WaitUntilCommand(() -> this.encoder.getAbsolutePosition() <= this.MIN_DEGREE + 1.0);
    }

    public Command isUpDone() {
        return new WaitUntilCommand(() -> this.encoder.getAbsolutePosition() >= this.MAX_DEGREE - 1.0);
    }

    public void lifterTo(double angle) {
        double speed = this.lifterPid.calculate(this.encoder.get(), angle);
        this.executeLifter(speed);
    }

    public Command onTrue() {
        return new ParallelDeadlineGroup(
            this.isDownDone(), Commands.runEnd(() -> this.lifterTo(this.MIN_DEGREE), this::stopLifter, this));
    }

    public Command onFalse() {
        return new ParallelDeadlineGroup(
            this.isUpDone(), Commands.runEnd(() -> this.lifterTo(this.MAX_DEGREE), this::stopLifter, this));
    }

    public void lifterAmp() {
        this.lifterTo(this.AMP_DEGREE);
    }

    public void stopLifter() {
        this.lifter.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Intake/position", this.encoder.getAbsolutePosition());
    }
}
