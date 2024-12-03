package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // TODO
    private final TalonModule front = new TalonModule(0, false, false);
    private final TalonModule back = new TalonModule(0, false, false);
    private final PIDController frontPid = new PIDController(0, 0, 0);
    private final Follower follower = new Follower(0, true);

    public ShooterSubsystem() {
        super("Shooter");
    }

    public void execute() {
        double frontSpeed = this.frontPid.calculate(this.front.getVelocity().getValue(), 50.0); // TODO
        this.front.set(frontSpeed);
        this.back.setControl(this.follower);
    }

    public boolean canShoot() {
        return (this.front.getVelocitySpeeds() + this.back.getVelocitySpeeds()) / 2.0 >= 50.0;
    }

    public void stopModules() {
        this.front.stopMotor();
        this.back.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter/Front", this.front.getVelocitySpeeds());
        SmartDashboard.putNumber("Shooter/Back", this.back.getVelocitySpeeds());
        SmartDashboard.putBoolean("Shooter/canShoot", this.canShoot());
    }
}
