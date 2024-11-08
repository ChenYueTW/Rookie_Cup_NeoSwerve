package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private final Driver driver = new Driver(0);
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(
		swerveSubsystem, driver::getXDesiredSpeed, driver::getYDesiredSpeed, driver::getRotationDesiredSpeed
	);
	private final ChoreoTrajectory trajectory = Choreo.getTrajectory("FiveNotes");
	
	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
	}

	public Command getAutonomousCommand() {
		Command choreo = Choreo.choreoSwerveCommand(
			this.trajectory,
			this.swerveSubsystem::getPose, 
			new PIDController(1.0, 0, 0),
			new PIDController(1.0, 0, 0),
			new PIDController(1.0, 0, 0),
			this.swerveSubsystem::chassisDrive,
			() -> {
                if (DriverStation.getAlliance().isPresent()) {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }
                return false;
            },
			this.swerveSubsystem
		);
		return Commands.sequence(
        	Commands.runOnce(() -> this.swerveSubsystem.resetPose(this.trajectory.getInitialPose())),
        	choreo,
        	this.swerveSubsystem.run(() -> this.swerveSubsystem.driveSwerve(0, 0, 0, false))
    	);
	}
}