package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.sencor.ColorSensor;
import frc.robot.lib.subsystems.SubsystemBase;

public class ConveySubsystem extends SubsystemBase {
    private final TalonModule convey = new TalonModule(0, false, false);
    private final ColorSensor colorSensor = new ColorSensor(Port.kOnboard);
    private final double CONVEY_SPEED = 0.0;

    public ConveySubsystem() {
        super("Convey");
    }

    public void execute() {
        this.convey.set(this.CONVEY_SPEED);
    }

    public void release() {
        this.convey.set(-this.CONVEY_SPEED);
    }

    public Command isAlliance() {
        return new WaitUntilCommand(() -> this.colorSensor.isAlliance());
    }
    
    public Command onTrue() {
        return new ParallelDeadlineGroup(
			this.isAlliance(),
			this.cmdExecute()).andThen(
				new ParallelRaceGroup(
					this.cmdRelease(),
					new WaitCommand(0.5))
            );
    }

    // public Command cmdExecute(double time) {
    //     return new ParallelRaceGroup(
    //         Commands.runEnd(this::execute, this::stopModules, this),
    //         new WaitCommand(time)
    //     );
    // }

    public Command cmdExecute() {
        return Commands.runEnd(this::execute, this::stopModules, this);
    }

    public Command cmdRelease() {
        return Commands.runEnd(this::release, this::stopModules, this);
    }

    public void stopModules() {
        this.convey.stopMotor();
    }

    @Override
    public void putDashboard() {
    }
}
