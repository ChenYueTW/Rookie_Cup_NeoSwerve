package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private final Supplier<Boolean> intake, release;

	public IntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Boolean> intake, Supplier<Boolean> release) {
		this.intakeSubsystem = intakeSubsystem;
		this.intake = intake;
		this.release = release;
		this.addRequirements(this.intakeSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (this.intake.get()) this.intakeSubsystem.executeIntake();
		else if (this.release.get()) this.intakeSubsystem.releaseIntake();
		else this.intakeSubsystem.stopIntake();
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeSubsystem.stopIntake();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
