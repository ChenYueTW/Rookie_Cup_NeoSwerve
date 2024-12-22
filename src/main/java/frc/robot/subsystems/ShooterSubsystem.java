package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // TODO
    private final TalonModule front = new TalonModule(15, true, false);
    private final TalonModule back = new TalonModule(16, false, false);
    private final Follower follower = new Follower(15, true);
    private final double GOAL_SPEED = 82.0;

    public ShooterSubsystem() {
        super("Shooter");
    }

    public void execute() {
        this.front.set(0.9);
        this.back.setControl(this.follower);
    }

    public Command autoShoot() {
        return Commands.runEnd(this::execute, this::stopModules, this);
    }

    public boolean canShoot() {
        return (this.front.getVelocityValue() + this.back.getVelocityValue()) / 2.0 >= this.GOAL_SPEED;
    }

    public void stopModules() {
        this.front.stopMotor();
        this.back.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Shooter/Front", this.front.getVelocityValue());
        SmartDashboard.putNumber("Shooter/Back", this.back.getVelocityValue());
        SmartDashboard.putBoolean("Shooter/canShoot", this.canShoot());
    }
}
