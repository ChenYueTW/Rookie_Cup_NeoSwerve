package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveConstants;

public class Controller extends XboxController {
    public Controller(int port) {
        super(port);
    }

    public double getShooterLifterSpeed() {
        return MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND);
    }
    public double getIntakeLifterSpeed() {
        return MathUtil.applyDeadband(this.getRightY(), SwerveConstants.DEAD_BAND);
    }
    public boolean getIntake() {
        return this.getAButton();
    }
    public boolean getIntakeRelease() {
        return this.getBButton();
    }
    public boolean getConvey() {
        return this.getXButton();
    }
    public boolean getConveyRelease() {
        return this.getXButton();
    }
    public boolean isShoot() {
        return this.getLeftBumper();
    }
}
