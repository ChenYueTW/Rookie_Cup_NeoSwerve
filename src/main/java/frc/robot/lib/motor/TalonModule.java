package frc.robot.lib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonModule extends TalonFX {
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

    public void rangeLimit(double max, double min, double speed) {
        if (this.getPositionDegrees() >= min && this.getPositionDegrees() <= max) {
            this.set(speed);
        } else if (this.getPositionDegrees() > max && speed <= 0.0) {
            this.set(speed);
        } else if (this.getPositionDegrees() < min && speed >= 0.0) {
            this.set(speed);
        } else {
            this.stopMotor();
        }
    }
}
