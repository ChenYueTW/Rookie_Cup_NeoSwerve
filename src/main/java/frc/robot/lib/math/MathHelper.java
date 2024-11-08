package frc.robot.lib.math;

import org.apache.commons.math3.util.FastMath;

import frc.robot.SwerveConstants;


public class MathHelper {
    public static double applyMax(double a, double max) {
        return FastMath.min(max, FastMath.abs(a)) * getSign(a);
    }
    public static double getSign(double a) {
        return (a == 0) ? 0 : (a / FastMath.abs(a));
    }
    public static double rpmToMpS(double rpm) {
        return SwerveConstants.WHEEL_RADIUS * rpm * (2 * Math.PI / 60.0);
    }
}
