package frc.robot.sensors;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import java.util.function.DoubleSupplier;

public class ElevatorLight extends SubsystemBase {

  private CANdle candle;

  // private String curAnimation = "default";
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(300);
  private final AddressableLEDBufferView rightSection = buffer.createView(0, 100);
  private final AddressableLEDBufferView topRightSection = buffer.createView(100, 150).reversed();
  private final AddressableLEDBufferView topLeftSection = buffer.createView(150, 200);
  private final AddressableLEDBufferView leftSection = buffer.createView(200, 300).reversed();
  private final AddressableLEDBufferView[] sections = {
    rightSection, topRightSection, topLeftSection, leftSection
  };

  // LED modes
  public RainbowAnimation rainbowAnim = new RainbowAnimation(.5, .89, 64);
  public LarsonAnimation larsonAnim =
      new LarsonAnimation(
          177, 156, 217); // try again with full string, looks like annoying flashing
  public FireAnimation fireAnim = new FireAnimation(); // oioioioi
  public StrobeAnimation strobeAnim =
      new StrobeAnimation(24, 15, 204); // idk it just shows as a single color, test again?

  public ElevatorLight() {

    candle = new CANdle(Hardware.ELEVATOR_LED);
    configureCandle();
    candle.clearAnimation(0);
    candle.setLEDs(255, 255, 255);
    // candle.animate(larsonAnim);
    // candle.animate(rainbowAnim);
    // candle.animate(fireAnim);
    // candle.animate(strobeAnim);
  }

  private void configureCandle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    candle.configAllSettings(config);
  }

  public Command colorSet(int r, int g, int b, String name) {
    return animate(LEDPattern.solid(new Color(r, g, b)), name);
  }

  public Command animate(LEDPattern animation, String name) {
    return run(() -> {
          for (AddressableLEDBufferView section : sections) {
            animation.applyTo(section);
            for (int i = 0; i < section.getLength(); ++i) {
              candle.setLEDs(section.getRed(i), section.getBlue(i), section.getGreen(i), 0, i, 1);
            }
          }
        })
        .withName("Animate" + name);
  }

  public LEDPattern greenProgress(DoubleSupplier progress) {
    LEDPattern base = LEDPattern.solid(Color.kGreen);
    LEDPattern mask = LEDPattern.progressMaskLayer(progress);
    LEDPattern heightDisplay = base.mask(mask);
    return heightDisplay;
  }
}
