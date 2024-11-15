package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motor.TalonModule;

public class ConveySubsystem extends SubsystemBase implements IDashboardProvider {
    private final TalonModule convey = new TalonModule(0, false, false);
    private final double CONVEY_SPEED = 0.0;

    public ConveySubsystem() {
        this.registerDashboard();
    }
    
    public void execute() {
        this.convey.set(this.CONVEY_SPEED);
    }

    public void release() {
        this.convey.set(-this.CONVEY_SPEED);
    }

    public Command cmdExecute(double time) {
        return new ParallelRaceGroup(
            Commands.runEnd(this::execute, this::stopModules, this),
            new WaitCommand(time)
        );
    }

    public void stopModules() {
        this.convey.stopMotor();
    }

    @Override
    public void putDashboard() {
    }
}