package frc.robot.lib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkModule extends CANSparkMax {
    private final RelativeEncoder encoder;
    private double gearRatio = 1.0;

    public SparkModule(int port, boolean reverse, boolean isBrake) {
        super(port, MotorType.kBrushless);
        this.restoreFactoryDefaults();
        this.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        this.setInverted(reverse);
        this.encoder = this.getEncoder();
    }

    public void setConversionFactor(double radius) {
        this.encoder.setVelocityConversionFactor(2.0 * radius * Math.PI / this.gearRatio / 60.0);
        this.encoder.setPositionConversionFactor(2.0 * radius * Math.PI / this.gearRatio);
    }

    public void setGearRatio(double gearRatio) {
        if (gearRatio == 0.0) {
            DriverStation.reportError("Gear Ratio can't set 0!", true);
            return;
        }
        this.gearRatio = gearRatio;
    }

    // RPM
    public double getVelocity() {
        return this.encoder.getVelocity();
    }

    // Degrees
    public double getPosition() {
        return this.encoder.getPosition();
    }
}
