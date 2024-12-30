package frc.robot.lib.math;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;
public class MathHelper {
    public static double applyMax(double a, double max) {
        return FastMath.min(max, FastMath.abs(a)) * getSign(a);
    }
    public static double getSign(double a) {
        return (a == 0) ? 0 : (a / FastMath.abs(a));
    }
    public static Vector2D rotatedVector(Vector2D vector, double angle) {
        Rotation rotation = new Rotation(new Vector3D(0.0, 0.0, 1.0), angle, RotationConvention.VECTOR_OPERATOR);
        Vector3D newVector = rotation.applyTo(new Vector3D(vector.getX(), vector.getY(), 0.0));
        return new Vector2D(newVector.getX(), newVector.getY());
    }
}
