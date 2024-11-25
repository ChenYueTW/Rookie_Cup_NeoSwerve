package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ShooterLifterSubsystem extends SubsystemBase {
    // TODO
    private final TalonModule shooterLifter = new TalonModule(0, false, true);
    private final PIDController lifterPid = new PIDController(0, 0, 0);
    private final double MAX_DEGREE = 0.0;
    private final double MIN_DEGREE = 0.0;

    public ShooterLifterSubsystem() {
        super("Shooter Liefter");
        this.shooterLifter.setDegreesRange(this.MAX_DEGREE, this.MIN_DEGREE);
    }

    public void execute(double speed) {
        this.shooterLifter.set(speed);
    }

    public void moveTo(double degrees) {
        double speed = this.lifterPid.calculate(this.shooterLifter.getPositionDegrees(), degrees);
        this.execute(speed);
    }

    public void stopModules() {
        this.shooterLifter.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter/Lifter Pos", this.shooterLifter.getPositionDegrees());
    }
}
