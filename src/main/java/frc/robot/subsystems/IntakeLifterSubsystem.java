package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
    private final PIDController lifterPid = new PIDController(0.0031, 0, 0); // TODO
    private final double MAX_DEGREE = 0.0;
    private final double MIN_DEGREE = -423.1273515781837;

    // private final Orchestra orchestra = new Orchestra();

    public IntakeLifterSubsystem() {
        super("Intake Liefter");
        this.encoder.reset();
        // this.orchestra.addInstrument(this.lifter);
        // this.orchestra.loadMusic("Mario.chrp");
        // this.orchestra.play();
    }

    public double getPosition() {
        return Units.rotationsToDegrees(-this.encoder.get());
    }

    /**
     * Limit the Intake lifter angle within the range.
     *
     * @param speed Set intake lifter speed.
     */
    public void executeLifter(double speed) {
        if (this.getPosition() >= this.MIN_DEGREE && this.getPosition() <= this.MAX_DEGREE) {
            this.lifter.set(speed);
        } else if (this.getPosition() > this.MAX_DEGREE && speed <= 0.0) {
            this.lifter.set(speed);
        } else if (this.getPosition() < this.MIN_DEGREE && speed >= 0.0) {
            this.lifter.set(speed);
        } else {
            this.lifter.stopMotor();
        }
    }

    /**
     * Detect whether the Intake lifter has reached the minimum angle
     * @return Command terminates until complete
     */
    public Command isDownDone() {
        return new WaitUntilCommand(() -> this.getPosition() <= this.MIN_DEGREE + 3.0);
    }

    /**
     * Detect whether the Intake lifter has reached the maximum angle
     * @return Command terminates until complete
     */
    public Command isUpDone() {
        return new WaitUntilCommand(() -> this.getPosition() >= this.MAX_DEGREE - 3.0);
    }

    public void lifterTo(double angle) {
        double speed = this.lifterPid.calculate(this.getPosition(), angle);
        this.executeLifter(speed);
    }

    public Command onTrue() {
        return new ParallelDeadlineGroup(
            this.isDownDone(),
            Commands.runEnd(() -> this.lifterTo(this.MIN_DEGREE), this::stopLifter, this));
    }

    public Command onFalse() {
        return new ParallelDeadlineGroup(
            this.isUpDone(),
            Commands.runEnd(() -> this.lifterTo(this.MAX_DEGREE), this::stopLifter, this));
    }

    public void stopLifter() {
        this.lifter.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Intake/position", this.getPosition());
    }
}
