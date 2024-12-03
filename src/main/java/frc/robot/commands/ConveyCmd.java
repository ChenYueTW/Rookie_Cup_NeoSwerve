package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.lib.subsystems.CommandBase;
import frc.robot.subsystems.ConveySubsystem;

public class ConveyCmd extends CommandBase {
	private final ConveySubsystem conveySubsystem;
	private final Supplier<Boolean> convey;

	public ConveyCmd(ConveySubsystem conveySubsystem, Supplier<Boolean> convey) {
		super(conveySubsystem);
		this.conveySubsystem = conveySubsystem;
		this.convey = convey;
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (this.convey.get()) this.conveySubsystem.execute();
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
