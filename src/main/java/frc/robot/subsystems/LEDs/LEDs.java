package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.VectorTools.util.HSV;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.subsystems.LEDs.LEDModes.Flash;
import frc.robot.subsystems.LEDs.LEDModes.OrangeDot;
import frc.robot.subsystems.LEDs.LEDModes.Rainbow;
import frc.robot.subsystems.LEDs.LEDModes.VectorWave;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDMode mode;

    private GamePiece hpSignal = GamePiece.CONE;

    private final OrangeDot orangeDot;
    private final Rainbow rainbow;
    private final VectorWave vectorWave;
    private final Flash purpleFlash;
    private final Flash yellowFlash;
    private final Flash greenFlash;
    private final Flash redFlash;

    public LEDs() {
        this.mode = Constants.LEDs.defaultMode;
        this.m_led = new AddressableLED(Constants.LEDs.id);
        this.m_ledBuffer = new AddressableLEDBuffer(Constants.LEDs.length);

        this.orangeDot = new OrangeDot(m_ledBuffer);
        this.rainbow = new Rainbow(m_ledBuffer);
        this.vectorWave = new VectorWave(m_ledBuffer);
        this.purpleFlash = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(263, 73, 96));
        this.yellowFlash = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(35, 100, 100));
        this.greenFlash = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(113, 100, 100));
        this.redFlash = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(0, 100, 100));

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        switch (mode) {
            case RAINBOW:
                rainbow.execute();
                break;
            case VECTORWAVE:
                vectorWave.execute();
                break;
            case ORANGEDOT:
                orangeDot.execute();
                break;
            case PURPLEFLASH:
                purpleFlash.execute();
                break;
            case YELLOWFLASH:
                yellowFlash.execute();
                break;
            case GREENFLASH:
                greenFlash.execute();
                break;
            case REDFLASH:
                redFlash.execute();
                break;
        }
        m_led.setData(m_ledBuffer);
        SmartDashboard.putString("LED Mode", mode.toString());
    }

    public void setLEDMode(LEDMode mode) {
        this.mode = mode;
    }

    public LEDMode getLEDMode() {
        return mode;
    }

    public void toggleHPSignal() {
        switch (hpSignal) {
            case CONE:
                hpSignal = GamePiece.CUBE;
                setLEDMode(LEDMode.PURPLEFLASH);
                break;
            case CUBE:
                hpSignal = GamePiece.CONE;
                setLEDMode(LEDMode.YELLOWFLASH);
                break;
        }
    }
}