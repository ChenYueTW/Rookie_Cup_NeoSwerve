package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.lib.subsystems.CommandBase;
import frc.robot.subsystems.ConveySubsystem;

public class ConveyCmd extends CommandBase {
	private final ConveySubsystem conveySubsystem;
	private final Supplier<Boolean> convey, release;

	public ConveyCmd(ConveySubsystem conveySubsystem, Supplier<Boolean> convey, Supplier<Boolean> release) {
		super(conveySubsystem);
		this.conveySubsystem = conveySubsystem;
		this.convey = convey;
		this.release = release;
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
