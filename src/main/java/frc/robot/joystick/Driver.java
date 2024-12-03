package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;

public class Driver extends XboxController implements IDashboardProvider {
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);
    public static boolean autoAimMode = false;

    public Driver(int port) {
        super(port);
        this.registerDashboard();
    }

    public double getXDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND) * 4.0 * this.getBrake();
        return this.xSpeedLimiter.calculate(speed);
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), SwerveConstants.DEAD_BAND) * 4.0 * this.getBrake();
        return this.ySpeedLimiter.calculate(speed);
    }

    public double getRotationDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), SwerveConstants.DEAD_BAND) * this.getBrake();
        return this.rotationLimiter.calculate(speed);
    }

    private double getBrake() {
        return 1.0 - MathUtil.applyDeadband(this.getLeftTriggerAxis(), SwerveConstants.DEAD_BAND);
    }

    public Trigger autoAimMode() {
        return new Trigger(this::getRightBumper);
    }

    public void transformAimMode() {
        autoAimMode = autoAimMode ? false : true;
    }

    public boolean getConveyInput() {
        return this.getXButton();
    }

    public boolean getConveyOutput() {
        return this.getYButton();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putBoolean("AimMode", autoAimMode);
    }
}
