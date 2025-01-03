package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends Command {
	private final ShooterSubsystem shooterSubsystem;
	private final Supplier<Boolean> isShoot;

	public ShooterCmd(ShooterSubsystem shooterSubsystem, Supplier<Boolean> isShoot) {
		this.shooterSubsystem = shooterSubsystem;
		this.isShoot = isShoot;
		this.addRequirements(this.shooterSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (this.isShoot.get()) this.shooterSubsystem.execute();
		else this.shooterSubsystem.stopModules();
	}

	@Override
	public void end(boolean interrupted) {
		this.shooterSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
