package frc.robot.lib.sencor;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor extends ColorSensorV3 {
    // TODO
    private final Color redCargo = new Color(0, 0, 0);
    private final Color blueCargo = new Color(0, 0, 0);

    public ColorSensor(Port port) {
        super(port);
        this.configureColorSensor(
            ColorSensorResolution.kColorSensorRes16bit,
            ColorSensorMeasurementRate.kColorRate25ms,
            GainFactor.kGain1x);
        this.configureProximitySensorLED(
            LEDPulseFrequency.kFreq100kHz,
            LEDCurrent.kPulse125mA, 255);
    }

    public boolean isGetBlue() {
        return this.getBlue() == this.blueCargo.blue && this.getGreen() == this.blueCargo.green
            && this.getRed() == this.blueCargo.red;
    }

    public boolean isGetRed() {
        return this.getBlue() == this.redCargo.blue && this.getGreen() == this.redCargo.green
            && this.getRed() == this.redCargo.red;
    }

    public Boolean isAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? this.isGetRed() : this.isGetBlue();
    }
}
