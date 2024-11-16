package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelecter {
    private static final SendableChooser<String> chooser = new SendableChooser<>();

    public enum Target {
        NOT_THING,
        ThreeNotes,
        FourNotes,
        FiveNotes,
        SixNotes,
        LeftSixNotes,
        EightNotes
    }

    public AutoModeSelecter() {
        chooser.setDefaultOption("NOT_THING", Target.NOT_THING.toString());
        chooser.addOption("ThreeNotes", Target.ThreeNotes.toString());
        chooser.addOption("FourNotes", Target.FourNotes.toString());
        chooser.addOption("FiveNotes", Target.FiveNotes.toString());
        chooser.addOption("SixNotes", Target.SixNotes.toString());
        chooser.addOption("LeftSixNotes", Target.LeftSixNotes.toString());
        chooser.addOption("EightNotes", Target.EightNotes.toString());
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public static SendableChooser<String> getChooser() {
        return chooser;
    }
}
