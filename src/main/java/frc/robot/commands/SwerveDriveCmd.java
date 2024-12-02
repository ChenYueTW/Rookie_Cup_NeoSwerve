package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.SwerveConstants;

public class SwerveDriveCmd extends Command {
	private final SwerveSubsystem swerveSubsystem;
	private final Supplier<Double> xSpeed, ySpeed, rotationSpeed;

	public SwerveDriveCmd(
		SwerveSubsystem swerveSubsystem,
		Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed
	) {
		this.swerveSubsystem = swerveSubsystem;
		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rotationSpeed = rotationSpeed;
		this.addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.swerveSubsystem.driveSwerve(this.xSpeed.get(), this.ySpeed.get(), this.rotationSpeed.get(), SwerveConstants.gyroField);

		if (RobotBase.isSimulation()) this.swerveSubsystem.simDrive(this.xSpeed.get(), this.ySpeed.get(), this.rotationSpeed.get());
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
