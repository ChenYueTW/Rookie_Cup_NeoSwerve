package frc.robot.lib.swerve;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class DriveEncoder implements RelativeEncoder {
    private final RelativeEncoder encoder;
    private final boolean reverse;

    public DriveEncoder(RelativeEncoder encoder, boolean reverse) {
        this.encoder = encoder;
        this.reverse = reverse;
    }

    @Override
    public double getPosition() {
        return this.encoder.getPosition() * (this.reverse ? 1 : -1);
    }

    @Override
    public double getVelocity() {
        return this.encoder.getVelocity() * (this.reverse ? 1 : -1);
    }

    @Override
    public REVLibError setPosition(double position) {
        return this.encoder.setPosition(position);
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        return this.encoder.setPositionConversionFactor(factor);
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        return this.encoder.setVelocityConversionFactor(factor);
    }

    @Override
    public double getPositionConversionFactor() {
        return this.encoder.getPositionConversionFactor();
    }

    @Override
    public double getVelocityConversionFactor() {
        return this.encoder.getVelocityConversionFactor();
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        return this.encoder.setAverageDepth(depth);
    }

    @Override
    public int getAverageDepth() {
        return this.encoder.getAverageDepth();
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        return this.encoder.setMeasurementPeriod(period_ms);
    }

    @Override
    public int getMeasurementPeriod() {
        return this.encoder.getMeasurementPeriod();
    }

    @Override
    public int getCountsPerRevolution() {
        return this.encoder.getCountsPerRevolution();
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        return this.encoder.setInverted(inverted);
    }

    @Override
    public boolean getInverted() {
        return this.encoder.getInverted();
    }
}
