package frc.robot.lib.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.IDashboardProvider;

public abstract class SubsystemBase extends edu.wpi.first.wpilibj2.command.SubsystemBase implements IDashboardProvider {
    private final SendableChooser<Boolean> debugChooser = new SendableChooser<>();
    private final boolean debug = false;

    public SubsystemBase(String name) {
        super(name);
        this.registerDashboard();
        RobotProvider.registerSubsystems(this);
        this.putDebugChooser();
    }

    public void putDebugChooser() {
        if (!debug) return;
        this.debugChooser.setDefaultOption("True", true);
        this.debugChooser.addOption("False", false);
        SmartDashboard.putData(this.getSubsystem(), this.debugChooser);
    }

    public boolean getChooser() {
        return this.debugChooser.getSelected();
    }

    @Override
    public abstract void putDashboard();
}
