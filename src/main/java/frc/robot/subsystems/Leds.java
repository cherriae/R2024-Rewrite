package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.AdvancedSubsystem;
import frc.robot.Constants.AddressableLed;

public class Leds extends AdvancedSubsystem {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private int currentPosition = 0;

  public Leds() {
    m_led = new AddressableLED(AddressableLed.LED_PORT);
    m_buffer = new AddressableLEDBuffer(AddressableLed.LED_COUNT);
    m_led.setLength(AddressableLed.LED_COUNT);

    m_led.start();
  }

  @Override
  public void periodic() {
    currentPosition = this.movingLEDEffect(0, 0, 255, 7, 1, currentPosition, 75);
    m_led.setData(m_buffer);
  }

  /**
   * Sets a single LED to a specific color using RGB values.
   *
   * @param index The LED index to set
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  public void setRGB(int index, int r, int g, int b) {
    if (index >= 0 && index < AddressableLed.LED_COUNT) {
      m_buffer.setRGB(index, r, g, b);
    }
  }

  /**
   * Sets a single LED to a specific color using HSV values.
   *
   * @param index The LED index to set
   * @param h Hue value (0-180)
   * @param s Saturation value (0-255)
   * @param v Value/brightness (0-255)
   */
  public void setHSV(int index, int h, int s, int v) {
    if (index >= 0 && index < AddressableLed.LED_COUNT) {
      m_buffer.setHSV(index, h, s, v);
    }
  }

  public void setAllRGB(int r, int g, int b) {
    for (int i = 0; i < AddressableLed.LED_COUNT; i++) {
      m_buffer.setRGB(i, r, g, b);
    }
  }

  public void setAllHSV(int h, int s, int v) {
    for (int i = 0; i < AddressableLed.LED_COUNT; i++) {
      m_buffer.setHSV(i, h, s, v);
    }
  }

  /**
   * Creates a moving LED effect across the strip.
   *
   * @param r Red value (0-255) for the moving LEDs
   * @param g Green value (0-255) for the moving LEDs
   * @param b Blue value (0-255) for the moving LEDs
   * @param length Number of LEDs lit in the moving group
   * @param speed Number of positions to move per call (can be negative for reverse direction)
   * @param position Current position of the first LED in the moving group
   * @return The new position after moving
   */
  public int movingLEDEffect(
      int r, int g, int b, int length, int speed, int position, int changeValue) {
    // Clear the strip
    turnOff();

    // Calculate new position
    position = (position + speed + AddressableLed.LED_COUNT) % AddressableLed.LED_COUNT;

    // Set the moving LEDs
    for (int i = 0; i < length; i++) {
      int pos = (position + i + AddressableLed.LED_COUNT) % AddressableLed.LED_COUNT;
      setRGB(pos, r - i * changeValue, g - i * changeValue, b - i * changeValue);
    }

    // Update the LED strip
    m_led.setData(m_buffer);

    // Return the new position
    return position;
  }

  /** Turns off all LEDs. */
  public void turnOff() {
    setAllRGB(0, 0, 0);
  }
}
