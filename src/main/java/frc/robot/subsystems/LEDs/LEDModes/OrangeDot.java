package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.VectorTools.util.HSV;
import frc.robot.Constants.LEDs;

/*
     * Orange dot
     */
public class OrangeDot extends LEDModeBase {
    private double m_orangeDotMiddleIndex = -LEDs.OrangeDot.pauseBetween;
    private HSV hsv = LEDs.OrangeDot.hsv;

    public OrangeDot(AddressableLEDBuffer m_ledBuffer) {
        super(m_ledBuffer);
    }

    public void execute() {
        for (int i = 0; i < m_ledBuffer.getLength() + LEDs.OrangeDot.pauseBetween; i++) {
            int value = MathUtil.clamp(
                    (int) ((1 / LEDs.OrangeDot.spread) * (Math.abs(m_orangeDotMiddleIndex - i)) * hsv.v)
                            - LEDs.OrangeDot.length,
                    0,
                    hsv.v) - hsv.v;

            if (m_ledBuffer.getLength() > i)
                m_ledBuffer.setHSV(i, hsv.h, hsv.s, value);
        }
        m_orangeDotMiddleIndex = (m_orangeDotMiddleIndex + LEDs.OrangeDot.speed) > m_ledBuffer.getLength()
                + LEDs.OrangeDot.pauseBetween ? -LEDs.OrangeDot.pauseBetween
                        : (m_orangeDotMiddleIndex + LEDs.OrangeDot.speed);
    }

}
