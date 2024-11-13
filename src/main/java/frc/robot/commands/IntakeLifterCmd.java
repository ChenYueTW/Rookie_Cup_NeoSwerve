package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeLifterSubsystem;

public class IntakeLifterCmd extends Command {
	private final IntakeLifterSubsystem intakeLifterSubsystem;
	private final Supplier<Double> lifter;

	public IntakeLifterCmd(IntakeLifterSubsystem intakeLifterSubsystem, Supplier<Double> lifter) {
		this.intakeLifterSubsystem = intakeLifterSubsystem;
		this.lifter = lifter;
		this.addRequirements(this.intakeLifterSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.intakeLifterSubsystem.executeLifter(this.lifter.get());
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeLifterSubsystem.stopLifter();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
