package frc.robot.lib.sensor;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor extends ColorSensorV3 {
    // TODO
    private final Color redCargo = new Color(255, 0, 0);
    private final Color blueCargo = new Color(0, 0, 255);
    private final ColorMatch colorMatch = new ColorMatch();

    public ColorSensor(Port port) {
        super(port);
        this.configureColorSensor(
            ColorSensorResolution.kColorSensorRes16bit,
            ColorSensorMeasurementRate.kColorRate25ms,
            GainFactor.kGain1x);
        this.configureProximitySensorLED(
            LEDPulseFrequency.kFreq100kHz,
            LEDCurrent.kPulse125mA, 255);
        this.colorMatch.addColorMatch(this.redCargo);
        this.colorMatch.addColorMatch(this.blueCargo);
    }

    public boolean isGetBlue() {
        double red = this.getColor().red;
        double green = this.getColor().green;
        double blue = this.getColor().blue;

        if (red >= 0.15 && red <= 0.25 && green >= 0.38 && green <= 0.5 && blue >= 0.3 && blue <= 0.45) {
            return true;
        }
        return false;
    }

    public boolean isGetRed() {
        double red = this.getColor().red;
        double green = this.getColor().green;
        double blue = this.getColor().blue;

        if (red >= 0.35 && red <= 0.38 && green >= 0.4 && green <= 0.45 && blue >= 0.15 && blue <= 0.22) {
            return true;
        }
        return false;
    }

    public Boolean isAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? this.isGetRed() : this.isGetBlue();
    }

    public String getRGB() {
        return this.getColor().red + ", " + this.getColor().green + ", " + this.getColor().blue;
    }
}
