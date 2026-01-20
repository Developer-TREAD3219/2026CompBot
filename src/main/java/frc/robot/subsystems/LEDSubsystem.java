package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;

/* 
 * TreadLeds provides different LED colors/patterns to indicate the state of
 * robot.
 */

enum State {
    Initial,
    Autonomous,
    TeleopInPosition,
    TeleopNotInPosition,
    Launching,
    Fueling,
    Climbing
}

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 9;
    private static final int kLength = 164; // 60 LEDs per Meter for 2.74 meters (27in x 4)

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLEDBufferView m_FrontView;
    private final AddressableLEDBufferView m_RightView;
    private final AddressableLEDBufferView m_LeftView;
    private final AddressableLEDBufferView m_BackView;

    public LEDSubsystem() {
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        int viewLength = kLength / 4; // a view for each side of the robot
        m_FrontView = m_buffer.createView(0, viewLength - 1);
        m_RightView = m_buffer.createView(viewLength, viewLength * 2 - 1);
        m_LeftView = m_buffer.createView(viewLength * 2, viewLength * 3 - 1);
        m_BackView = m_buffer.createView(viewLength * 3, viewLength * 4 - 1);
        m_led.setLength(kLength);
        m_led.start();

        // Set the default command to turn the strip off, otherwise the last colors
        // written by
        // the last command to run will continue to be displayed.
        // Note: Other default patterns could be used instead!
        setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }

    @Override
    public void periodic() {
        // Periodically send the latest LED color data to the LED strip for it to
        // display
        m_led.setData(m_buffer);
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_buffer));
    }

    public LEDPattern setState(State state) {
        LEDPattern pattern;
        switch (state) {
            case Initial:
                pattern = getTread3219(); // Go Green and Gold!
                break;
            case Autonomous:
                pattern = getSolidColor(0, 0, 255); // Blue
                break;
            case TeleopInPosition:
                pattern = getSolidColor(0, 255, 0); // Green
                break;
            case Launching:
                pattern = getBlinkingColor(255, 0, 255); // Purple
                break;
            case Fueling:
                pattern = getBlinkingColor(0, 255, 255); // Cyan
                break;
            case TeleopNotInPosition:
                pattern = getBlinkingColor(255, 255, 0); // Yellow
                break;
            case Climbing:
                pattern = getBlinkingColor(255, 0, 0); // Red
                break;
            default:
                pattern = getSolidColor(0, 0, 0); // Off
        }
        return pattern;
    }

    private LEDPattern getTread3219() {
        LEDPattern rainbow = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen, Color.kYellow);

        return rainbow;
    }

    private LEDPattern getBlinkingColor(int r, int g, int b) {
        LEDPattern base = LEDPattern.solid(new Color(r, g, b));
        LEDPattern blinkColor = base.blink(Units.Seconds.of(0.5));
        return blinkColor;
    }

    private LEDPattern getSolidColor(int r, int g, int b) {
        LEDPattern rgbColor = LEDPattern.solid(new Color(r, g, b));
        return rgbColor;
    }

    public void TestCycleStates() {
        // Cycle through all states for testing
        for (State state : State.values()) {
            setState(state);
            try {
                Thread.sleep(5000); // Wait for 2 seconds
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
