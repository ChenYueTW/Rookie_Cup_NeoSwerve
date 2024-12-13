package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SwerveConstants;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class IntakeLifterSubsystem extends SubsystemBase {
    private final TalonModule lifter = new TalonModule(13, false, true);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController lifterPid = new PIDController(0.8, 0.0, 0.0); // TODO
    private final double MAX_DEGREE = 0.0083333333333333;
    private final double MIN_DEGREE = -0.347941295713238; // TODO
    private final double GEAR_RATIO = 5.0 / 17.0;

    public IntakeLifterSubsystem() {
        super("Intake Liefter");
        this.encoder.reset();
    }

    /** 
     * Units: degree
     * 
     * @return intake lifter angle.
     */
    public double getPosition() {
        return -this.encoder.get() * this.GEAR_RATIO;
    }

    /**
     * Limit the Intake lifter angle within the range.
     *
     * @param speed Set intake lifter speed.
     */
    public void executeLifter(double speed) {
        if (speed > 0.5) speed = 0.5;
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
     * Detect whether the Intake lifter has reached the minimum angle.
     * 
     * @return Command terminates until complete.
     */
    public Command isDownDone() {
        return new WaitUntilCommand(() -> this.getPosition() <= this.MIN_DEGREE + 0.01);
    }

    /**
     * Detect whether the Intake lifter has reached the maximum angle.
     * 
     * @return Command terminates until complete.
     */
    public Command isUpDone() {
        return new WaitUntilCommand(() -> this.getPosition() >= this.MAX_DEGREE - 0.02);
    }

    /**
     * Move intake lifter to angle
     *
     * @param angle The angle to which the intake lifter is to be moved.
     */
    public void liftTo(double angle) {
        double speed = MathUtil.applyDeadband(this.lifterPid.calculate(this.getPosition(), angle), SwerveConstants.DEAD_BAND);

        this.executeLifter(speed);
        SmartDashboard.putNumber("speed", speed);
    }

    /**
     * Intake lifter goes down until isDownDone() is triggered.
     * 
     * @return Command has finished.
     */
    public Command onTrue() {
        return new ParallelDeadlineGroup(
            this.isDownDone(),
            Commands.runEnd(() -> this.liftTo(this.MIN_DEGREE), this::stopLifter, this));
    }

    /**
     * Intake lifter goes down until isUpDone() is triggered.
     * 
     * @return Command has finished.
     */
    public Command onFalse() {
        return new ParallelDeadlineGroup(
            this.isUpDone(),
            Commands.runEnd(() -> this.liftTo(this.MAX_DEGREE), this::stopLifter, this));
    }

    public void stopLifter() {
        this.lifter.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Intake/position", -this.getPosition());
    }
}
