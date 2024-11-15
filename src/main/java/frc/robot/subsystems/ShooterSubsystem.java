package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.TalonModule;

public class ShooterSubsystem extends SubsystemBase implements IDashboardProvider {
    // TODO
    private final TalonModule front = new TalonModule(0, false, false);
    private final TalonModule back = new TalonModule(0, false, false);
    private final PIDController frontPid = new PIDController(0, 0, 0);
    private final Follower follower = new Follower(0, true);

    public ShooterSubsystem() {
        this.registerDashboard();
    }

    public void execute() {
        double frontSpeed = this.frontPid.calculate(this.front.getVelocity().getValue(), 0); // TODO
        this.front.set(frontSpeed);
        this.back.setControl(this.follower);
    }

    public void stopModules() {
        this.front.stopMotor();
        this.back.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter/Front", this.front.getPositionDegrees());
        SmartDashboard.putNumber("Shooter/Back", this.back.getPositionDegrees());
    }
}
