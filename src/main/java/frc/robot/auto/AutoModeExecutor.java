package frc.robot.auto;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoModeExecutor {
    private final SwerveSubsystem swerveSubsystem;

    public AutoModeExecutor(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public Command getAutonomousCommand() {
        if (AutoModeSelecter.getChooser().getSelected() == "NOT_THING") return null;

        this.swerveSubsystem.resetPose(Choreo.getTrajectory(AutoModeSelecter.getChooser().getSelected()).getInitialPose());

		return Choreo.choreoSwerveCommand(
			Choreo.getTrajectory(AutoModeSelecter.getChooser().getSelected()),
			this.swerveSubsystem::getPose,
			new PIDController(0.0001, 0.001, 0),
			new PIDController(0.0001, 0001, 0),
			new PIDController(0.001, 0.01, 0),
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
}
