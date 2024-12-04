package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveConstants;

public class Controller extends XboxController {
    public Controller() {
        super(1);
    }

    public double getIntakeLifterSpeed() {
        return -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND) * 0.3;
    }

    public double geShooterLifterSpeed() {
        return MathUtil.applyDeadband(this.getRightY(), SwerveConstants.DEAD_BAND);
    }

    public Trigger getConveyInput() {
        return new Trigger(this::getXButton);
    }

    public Trigger autoIntake() {
        return new Trigger(this::getAButton);
    }

    public boolean intake() {
        return this.getYButton();
    }

    public boolean releaseIntake() {
        return this.getBButton();
    }

    public Trigger shoot() {
        return new Trigger(this::getLeftBumper);
    }

    public Trigger autoShoot() {
        return new Trigger(this::getRightBumper);
    }
}
