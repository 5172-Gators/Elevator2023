package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Rainbow extends LEDModeBase {
    private int m_rainbowFirstPixelHue = 0;
    public Rainbow(AddressableLEDBuffer m_ledBuffer) {
        super(m_ledBuffer);
    }

    public void execute() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

}
