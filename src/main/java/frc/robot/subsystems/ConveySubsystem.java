package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.motor.SparkModule;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.sensor.ColorSensor;
import frc.robot.lib.subsystems.SubsystemBase;

public class ConveySubsystem extends SubsystemBase {
    private final SparkModule convey = new SparkModule(17, true, false);
    private final ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);
    private final double CONVEY_SPEED = 0.3;

    public ConveySubsystem() {
        super("Convey");
    }

    public void execute() {
        this.convey.set(this.CONVEY_SPEED);
    }

    public void release() {
        this.convey.set(-this.CONVEY_SPEED);
    }

    public Command isAllianceCmd() {
        return new WaitUntilCommand(() -> this.colorSensor.isAlliance());
    }

    public Command isRed() {
        return new WaitUntilCommand(() -> this.colorSensor.isGetRed());
    }

    public boolean isAlliance() {
        return this.colorSensor.isAlliance();
    }
    
    public Command onTrue() {
        return new ParallelRaceGroup(
            Commands.run(this::execute, this),
            this.isRed()
        );
    }

    public Command cmdExecute() {
        return Commands.runEnd(this::execute, this::stopModules, this);
    }

    public Command cmdRelease() {
        return new ParallelRaceGroup(
            Commands.runEnd(this::release, this::stopModules, this),
            new WaitCommand(0.08)
        );
    }

    public Command shootRelease() {
        return new ParallelRaceGroup(
            Commands.runEnd(() -> this.convey.set(0.7), this::stopModules, this),
            new WaitCommand(1.0)
        );
    }

    public void stopModules() {
        this.convey.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putBoolean("Convey/isAlliance", this.colorSensor.isAlliance());
        SmartDashboard.putBoolean("Convey/isGetBlue", this.colorSensor.isGetBlue());
        SmartDashboard.putBoolean("Convey/isGetRed", this.colorSensor.isGetRed());
        SmartDashboard.putString("RGB", this.colorSensor.getRGB());
    }
}
