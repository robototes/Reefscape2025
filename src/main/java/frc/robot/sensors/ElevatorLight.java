package frc.robot.sensors;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ElevatorLight extends SubsystemBase {

  private CANdle candle;

  // private String curAnimation = "default";

  // LED modes
  public RainbowAnimation rainbowAnim = new RainbowAnimation(.5, .89, 64);
  public LarsonAnimation larsonAnim =
      new LarsonAnimation(
          177, 156, 217); // try again with full string, looks like annoying flashing
  public TwinkleAnimation twinkleAnim =
      new TwinkleAnimation(135, 30, 270); // cool ig idk test on full string, it twinkle
  public ColorFlowAnimation colorFlowAnim =
      new ColorFlowAnimation(40, 14, 15); // uh, it just flashes idk man
  public FireAnimation fireAnim = new FireAnimation(); // dont just dont bruzz
  public RgbFadeAnimation rgbFadeAnim =
      new RgbFadeAnimation(); // can we change it? annoying ngl just use rainbow, literally RBG
  public SingleFadeAnimation singleFadeAnim =
      new SingleFadeAnimation(40, 14, 15); // one color lowkey boring, more of a blink than a flash
  public StrobeAnimation strobeAnim =
      new StrobeAnimation(24, 15, 204); // idk it just shows as a single color, test again?

  public ElevatorLight() {

    candle = new CANdle(Hardware.ELEVATOR_LED);
    configureCandle();
    candle.clearAnimation(0);
    candle.setLEDs(255, 255, 255);
    // candle.animate(larsonAnim);
    // candle.animate(rainbowAnim);
    // candle.animate(twinkleAnim);
    // candle.animate(colorFlowAnim);
    // candle.animate(fireAnim);
    // candle.animate(rgbFadeAnim);
    // candle.animate(singleFadeAnim);
    // candle.animate(strobeAnim);
  }

  private void configureCandle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    candle.configAllSettings(config);
  }

  public Command colorSet(int r, int g, int b) {
    return runOnce(
        () -> {
          candle.clearAnimation(0);
          candle.setLEDs(r, g, b);
        });
  }

  public Command animate(Animation animation) {
    return runOnce(() -> candle.animate(animation));
  }
}
