package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelecter {
    private static final SendableChooser<Target> chooser = new SendableChooser<>();

    public enum Target {
        NOT_THING,
        THREE_NOTES,
        FOURNOTES,
        FIVENOTES
    }

    public AutoModeSelecter() {
        chooser.addOption("NOT_THING", Target.NOT_THING);
        chooser.addOption("ThreeNotes", Target.THREE_NOTES);
        chooser.addOption("FourNotes", Target.FOURNOTES);
        chooser.addOption("FiveNotes", Target.FIVENOTES);
        SmartDashboard.putData(chooser);
    }

    public static SendableChooser<Target> getChooser() {
        return chooser;
    }
}
