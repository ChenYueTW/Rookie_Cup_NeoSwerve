package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.AutoModeSelecter;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private final Driver driver = new Driver(0);
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(
		swerveSubsystem, driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationDesiredSpeed
	);

	// Auto
	@SuppressWarnings("unused")
	private final AutoModeSelecter autoModeSelecter = new AutoModeSelecter();
	private final AutoModeExecutor autoModeExecutor = new AutoModeExecutor(swerveSubsystem);
	
	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
	}

	public Command getAutonomousCommand() {
		return this.autoModeExecutor.getAutonomousCommand();
	}
}
