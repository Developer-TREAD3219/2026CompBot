package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class TreadLeds {

    
    public static final int kLedCnt = 162;
    
    AddressableLED m_led = new AddressableLED(9);

    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedCnt);

    public TreadLeds() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
    }

}
