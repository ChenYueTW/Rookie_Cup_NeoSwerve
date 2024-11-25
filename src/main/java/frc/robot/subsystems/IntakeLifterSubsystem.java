package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.encoder.RevEncoder;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class IntakeLifterSubsystem extends SubsystemBase {
    private final TalonModule lifter = new TalonModule(0, false, true);
    private final RevEncoder encoder = new RevEncoder(0);
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

    public void lifterTo(double angle) {
        double speed = this.lifterPid.calculate(this.encoder.getRpmPosition(), angle);
        this.executeLifter(speed);
    }

    public void lifterAmp() {
        this.lifterTo(this.AMP_DEGREE);
    }

    public void stopLifter() {
        this.lifter.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Lifter Pos", this.encoder.getRpmPosition());
    }
}
