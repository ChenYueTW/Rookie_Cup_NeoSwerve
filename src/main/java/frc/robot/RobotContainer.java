package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.AutoModeSelecter;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeLifterCmd;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Controller;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.ConveySubsystem;
import frc.robot.subsystems.IntakeLifterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private final Driver driver = new Driver();
	private final Controller controller = new Controller();

	// private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final ConveySubsystem conveySubsystem = new ConveySubsystem();
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final IntakeLifterSubsystem intakeLifterSubsystem = new IntakeLifterSubsystem();

	// private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(
		// swerveSubsystem, driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationDesiredSpeed);
	private final IntakeCmd intakeCmd = new IntakeCmd(intakeSubsystem, controller::intake, controller::releaseIntake);
	private final IntakeLifterCmd intakeLifterCmd = new IntakeLifterCmd(intakeLifterSubsystem, controller::getIntakeLifterSpeed);

	// Auto
	@SuppressWarnings("unused")
	private final AutoModeSelecter autoModeSelecter = new AutoModeSelecter();
	// private final AutoModeExecutor autoModeExecutor = new AutoModeExecutor(swerveSubsystem);

	public RobotContainer() {
		// this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
		this.intakeSubsystem.setDefaultCommand(this.intakeCmd);
		this.intakeLifterSubsystem.setDefaultCommand(this.intakeLifterCmd);

		this.configBindings();
	}

	public void configBindings() {
		// Use auto aim mode to  aim HUB
		// this.driver.autoAimMode()
			// .onTrue(Commands.runOnce(this.driver::transformAimMode, this.swerveSubsystem));

		// input cargo and if color sensor detected that, back.
		// this.controller.getConveyInput()
		// 	.onTrue(this.conveySubsystem.onTrue()
		// 		.andThen(this.conveySubsystem.cmdRelease()))
		// 	.onFalse(Commands.run(this.conveySubsystem::stopModules, this.conveySubsystem));

		// Shoot cargo
		// this.controller.shoot()
			// .whileTrue(
				// Commands.runEnd(this.shooterSubsystem::execute, this.shooterSubsystem::stopModules, this.shooterSubsystem));

		// intake part auto
		this.controller.autoIntake()
			.onTrue(this.intakeLifterSubsystem.onTrue()
				.andThen(new ParallelCommandGroup(
					this.intakeSubsystem.cmdExecute()
					// this.conveySubsystem.onTrue()
				)))
				// .until(() -> this.conveySubsystem.isAlliance())))
			.onFalse(this.intakeLifterSubsystem.onFalse()
				.andThen(new ParallelCommandGroup(
					Commands.runOnce(this.intakeSubsystem::stopIntake, this.intakeSubsystem)
					// Commands.runOnce(this.conveySubsystem::stopModules, this.conveySubsystem)
				).raceWith(new WaitCommand(2.0))));
		
		// Auto Shoot
		this.controller.autoShoot()
			.onTrue(this.shooterSubsystem.autoShoot()
				.raceWith(
					new WaitUntilCommand(() -> this.shooterSubsystem.canShoot())
						.andThen(this.conveySubsystem.cmdRelease())
				));
	}

	public Command getAutonomousCommand() {
		// return this.autoModeExecutor.getAutonomousCommand();
		return null;
	}
}
