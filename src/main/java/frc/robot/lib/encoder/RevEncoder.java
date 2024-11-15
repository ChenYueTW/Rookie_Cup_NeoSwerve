package frc.robot.lib.encoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RevEncoder extends DutyCycleEncoder {
    public RevEncoder(int port) {
        super(port);
        this.reset();
    }

    // Max & Min is Degrees
    public int isInRange(double max, double min) {
        if (this.getAbsolutePosition() > max) return 1; // Above Range
        else if (this.getAbsolutePosition() <= max && this.getAbsolutePosition() >= min) return 0; // In Range
        else return -1; // Under Range
    }

    public double getRpmPosition() {
        return Units.rotationsToDegrees(this.getAbsolutePosition());
    }
}
