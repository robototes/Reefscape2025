package frc.robot.sensors;

import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.util.ScoringMode;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ElevatorLight extends SubsystemBase {

  private CANdle candle;

  // private String curAnimation = "default";
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(71);
  private final AddressableLEDBufferView[] sections = {buffer.createView(0, 70)};

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

  public Command tripleBlink(int r, int g, int b, String name) {
    return animate(LEDPattern.solid(new Color(r, g, b)).blink(Seconds.of(1.0 / 6.0)), name)
        .withTimeout(Seconds.of(1.0))
        .withName("Animate" + name);
  }

  public Command pulse(int r, int g, int b, String name) {
    return animate(LEDPattern.solid(new Color(r, g, b)).breathe(Seconds.of(1.0)), name);
  }

  public Command animate(LEDPattern animation, String name) {
    // Always proxy so that compositions using LEDs will not get interrupted by another composition
    // using LEDs
    return run(() -> {
          updateLEDs(animation);
        })
        .asProxy()
        .withName("Animate" + name);
  }

  public Command showScoringMode(Supplier<ScoringMode> scoringMode) {
    // This is only used as the default command, so don't proxy
    // If we need to use it elsewhere for some reason, add the proxy there
    return run(() -> {
          ScoringMode currentMode = scoringMode.get();
          if (currentMode == ScoringMode.ALGAE) {
            updateLEDs(LEDPattern.solid(Color.kTeal));
          } else {
            updateLEDs(LEDPattern.solid(Color.kWhite));
          }
        })
        .withName("Animate Scoring Mode");
  }

  private void updateLEDs(LEDPattern animation) {
    for (AddressableLEDBufferView section : sections) {
      animation.applyTo(section);
      for (int i = 0; i < section.getLength(); ++i) {
        candle.setLEDs(section.getRed(i), section.getGreen(i), section.getBlue(i), 0, i, 1);
      }
    }
  }

  public LEDPattern greenProgress(DoubleSupplier progress) {
    LEDPattern base = LEDPattern.solid(Color.kGreen);
    LEDPattern mask = LEDPattern.progressMaskLayer(progress);
    LEDPattern heightDisplay = base.mask(mask);
    return heightDisplay;
  }
}
