package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveConstants;

public class Driver extends XboxController{
    SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);

    public Driver(int port) {
        super(port);
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
}
