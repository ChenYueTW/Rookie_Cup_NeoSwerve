package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ShooterLifterSubsystem extends SubsystemBase {
    // TODO
    private final TalonModule shooterLifter = new TalonModule(18, false, true);
    private final PIDController lifterPid = new PIDController(0, 0, 0);
    private final double GEAR_RATIO = 0.0;
    private final double MAX_DEGREE = 0.0;
    private final double MIN_DEGREE = 0.0;

    public ShooterLifterSubsystem() {
        super("Shooter Liefter");
    }

    public double getPosition() {
        return this.shooterLifter.getPositionValue() * this.GEAR_RATIO;
    }

    public void execute(double speed) {
        this.shooterLifter.set(speed);
    }

    public void liftTo(double degrees) {
        double speed = this.lifterPid.calculate(this.getPosition(), degrees);
        this.execute(speed);
    }

    public void stopModules() {
        this.shooterLifter.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter/Lifter Pos", this.getPosition());
    }
}
