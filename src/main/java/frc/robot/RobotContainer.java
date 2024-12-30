package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.AutoModeSelecter;
import frc.robot.commands.ConveyCmd;
import frc.robot.commands.IntakeLifterCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.ShooterLifterCmd;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Controller;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.ConveySubsystem;
import frc.robot.subsystems.IntakeLifterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterLifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private final Driver driver = new Driver();
	private final Controller controller = new Controller();

	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final ConveySubsystem conveySubsystem = new ConveySubsystem();
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	// private final ShooterLifterSubsystem shooterLifterSubsystem = new ShooterLifterSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final IntakeLifterSubsystem intakeLifterSubsystem = new IntakeLifterSubsystem();

	private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(
		swerveSubsystem, driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationDesiredSpeed, driver::resetGyro);
	private final ShooterCmd shooterCmd = new ShooterCmd(shooterSubsystem, controller::isShoot);
	// private final ShooterLifterCmd shooterLifterCmd = new ShooterLifterCmd(shooterLifterSubsystem, controller::geShooterLifterSpeed);
	private final IntakeLifterCmd intakeLifterCmd = new IntakeLifterCmd(intakeLifterSubsystem, controller::getIntakeLifterSpeed);

	private final VisionSubsystem visionSubsystem = new VisionSubsystem();
	// Auto
	@SuppressWarnings("unused")
	private final AutoModeSelecter autoModeSelecter = new AutoModeSelecter();
	private final AutoModeExecutor autoModeExecutor = new AutoModeExecutor(swerveSubsystem);

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
		this.shooterSubsystem.setDefaultCommand(this.shooterCmd);
		// this.shooterLifterSubsystem.setDefaultCommand(this.shooterLifterCmd);
		this.intakeLifterSubsystem.setDefaultCommand(this.intakeLifterCmd);

		this.configBindings();
	}

	public void configBindings() {
		// Use auto aim mode to aim HUB
		this.driver.autoAimMode()
			.onTrue(Commands.runOnce(this.driver::transformAimMode, this.swerveSubsystem));
		// this.driver.autoTrackCargo()
			// .onTrue(
				// Commands.runEnd(() -> this.swerveSubsystem.situateRobot(VisionSubsystem.getCargo2d(), VisionSubsystem.getCargoAngle()), this.swerveSubsystem::stopModules, this.swerveSubsystem));

		// TODO
		// input cargo and if color sensor detected that, back.
		this.controller.getConveyInput()
			.onTrue(this.conveySubsystem.cmdExecute()
				.raceWith(this.conveySubsystem.isAllianceCmd())
				.andThen(this.conveySubsystem.cmdRelease()))
			.onFalse(Commands.run(this.conveySubsystem::stopModules, this.conveySubsystem));

		// intake part auto
		this.controller.autoIntake()
			.onTrue(this.intakeLifterSubsystem.onTrue()
				.andThen(new ParallelCommandGroup(
					this.intakeSubsystem.cmdExecute(),
					this.conveySubsystem.cmdExecute()
						.raceWith(this.conveySubsystem.isAllianceCmd())
						.andThen(this.conveySubsystem.cmdRelease())
				)))
			.onFalse(this.intakeLifterSubsystem.onFalse()
				.andThen(new ParallelCommandGroup(
					Commands.runOnce(this.intakeSubsystem::stopIntake, this.intakeSubsystem),
					Commands.runOnce(this.conveySubsystem::stopModules, this.conveySubsystem)
				).raceWith(new WaitCommand(2.0))));
		
		// Auto Shoot
		this.controller.autoShoot()
			.onTrue(this.shooterSubsystem.autoShoot()
				.raceWith(
					new WaitUntilCommand(() -> this.shooterSubsystem.canShoot())
						.andThen(this.conveySubsystem.shootRelease())
				));
		// Intake IN
		this.controller.intake()
			.onTrue(this.intakeSubsystem.cmdExecute())
			.onFalse(Commands.runOnce(this.intakeSubsystem::stopIntake, this.intakeSubsystem));
		// Intake Release
		this.controller.releaseIntake()
			.onTrue(this.intakeSubsystem.cmdRelease())
			.onFalse(Commands.runOnce(this.intakeSubsystem::stopIntake, this.intakeSubsystem));
	}

	public Command getAutonomousCommand() {
		// return this.autoModeExecutor.getAutonomousCommand();
		return null;
	}
}
