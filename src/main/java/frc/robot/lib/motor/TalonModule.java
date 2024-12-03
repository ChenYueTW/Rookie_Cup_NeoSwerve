package frc.robot.lib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonModule extends TalonFX {
    private double maxDegrees = 0.0;
    private double minDegrees = 0.0;

    public TalonModule(int port, boolean reverse, boolean isBrake) {
        super(port);
        this.clearStickyFaults();
        this.setInverted(reverse);
        this.setNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(35.0)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(35.0)
            .withSupplyCurrentThreshold(40.0)
            .withSupplyTimeThreshold(40.0);

        TalonFXConfiguration FXConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);
        this.getConfigurator().refresh(FXConfig);
        this.setPosition(0.0);
    }

    public double getPositionDegrees() {
        return this.getPosition().getValue();
    }

    public double getVelocitySpeeds() {
        return this.getVelocity().getValue();
    }

    public void setDegreesRange(double max, double min) {
        this.maxDegrees = max;
        this.minDegrees = min;
    }


    @Override
    public void set(double speed) {
        if (this.minDegrees == 0.0 && this.maxDegrees == 0.0) {
            this.set(speed);
            return;
        }
        if (this.getPositionDegrees() >= this.minDegrees && this.getPositionDegrees() <= this.maxDegrees) {
            this.set(speed);
        } else if (this.getPositionDegrees() > this.maxDegrees && speed <= 0.0) {
            this.set(speed);
        } else if (this.getPositionDegrees() < this.minDegrees && speed >= 0.0) {
            this.set(speed);
        } else {
            this.stopMotor();
        }
    }
}
