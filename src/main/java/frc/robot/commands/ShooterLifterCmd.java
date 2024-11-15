package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLifterSubsystem;

public class ShooterLifterCmd extends Command {
	private final ShooterLifterSubsystem shooterLifterSubsystem;
	private final Supplier<Double> speed;

	public ShooterLifterCmd(ShooterLifterSubsystem shooterLifterSubsystem, Supplier<Double> speed) {
		this.shooterLifterSubsystem = shooterLifterSubsystem;
		this.speed = speed;
		this.addRequirements(this.shooterLifterSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.shooterLifterSubsystem.execute(this.speed.get());
	}

	@Override
	public void end(boolean interrupted) {
		this.shooterLifterSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
