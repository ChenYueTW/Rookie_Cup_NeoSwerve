package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelecter {
    private static final SendableChooser<String> chooser = new SendableChooser<>();

    public enum Target {
        NOT_THING,
        THREE_NOTES,
        FOURNOTES,
        FIVENOTES
    }

    public AutoModeSelecter() {
        chooser.setDefaultOption("DO_NOTTHING", Target.NOT_THING.toString());
        chooser.addOption("ThreeNotes", Target.THREE_NOTES.toString());
        chooser.addOption("FourNotes", Target.FOURNOTES.toString());
        chooser.addOption("FiveNotes", Target.FIVENOTES.toString());
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public static SendableChooser<String> getChooser() {
        return chooser;
    }
}
