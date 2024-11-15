package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveySubsystem;

public class ConveyCmd extends Command {
	private final ConveySubsystem conveySubsystem;
	private final Supplier<Boolean> convey, release;

	public ConveyCmd(ConveySubsystem conveySubsystem, Supplier<Boolean> convey, Supplier<Boolean> release) {
		this.conveySubsystem = conveySubsystem;
		this.convey = convey;
		this.release = release;
		this.addRequirements(this.conveySubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (this.convey.get()) this.conveySubsystem.execute();
		else if (this.release.get()) this.conveySubsystem.release();
		else this.conveySubsystem.stopModules();
	}

	@Override
	public void end(boolean interrupted) {
		this.conveySubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
