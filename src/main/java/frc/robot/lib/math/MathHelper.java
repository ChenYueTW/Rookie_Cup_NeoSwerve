package frc.robot.lib.math;

import org.apache.commons.math3.util.FastMath;
public class MathHelper {
    public static double applyMax(double a, double max) {
        return FastMath.min(max, FastMath.abs(a)) * getSign(a);
    }
    public static double getSign(double a) {
        return (a == 0) ? 0 : (a / FastMath.abs(a));
    }
}
