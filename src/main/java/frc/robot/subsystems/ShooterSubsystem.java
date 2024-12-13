package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // TODO
    private final TalonModule front = new TalonModule(0, false, false);
    private final TalonModule back = new TalonModule(0, false, false);
    private final PIDController frontPid = new PIDController(0, 0, 0);
    private final Follower follower = new Follower(0, true);
    private final double GOAL_SPEED = 50.0;

    public ShooterSubsystem() {
        super("Shooter");
    }

    public void execute() {
        double frontSpeed = this.frontPid.calculate(this.front.getVelocity().getValue(), this.GOAL_SPEED); // TODO
        this.front.set(frontSpeed);
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
