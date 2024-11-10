package frc.robot.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoModeExecutor {
    private final ChoreoTrajectory trajectory;
    private final SwerveSubsystem swerveSubsystem;
    private final Command choreo;

    public AutoModeExecutor(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.trajectory = Choreo.getTrajectory(AutoModeSelecter.getChooser().toString());

        // TODO
        this.choreo = Choreo.choreoSwerveCommand(
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
    }

    public Command getAutonomousCommand() {
		return Commands.sequence(
        	Commands.runOnce(() -> this.swerveSubsystem.resetPose(this.trajectory.getInitialPose())),
        	this.choreo,
        	Commands.runOnce(this.swerveSubsystem::stopModules, this.swerveSubsystem)
    	);
    }
}
