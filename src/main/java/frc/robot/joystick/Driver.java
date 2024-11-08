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
        double speed = -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND);
        return this.xSpeedLimiter.calculate(speed);
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), SwerveConstants.DEAD_BAND);
        return this.ySpeedLimiter.calculate(speed);
    }

    public double getRotationDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), SwerveConstants.DEAD_BAND);
        return this.rotationLimiter.calculate(speed);
    }
}
