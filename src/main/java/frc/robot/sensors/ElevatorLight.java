package frc.robot.sensors;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ElevatorLight extends SubsystemBase {

  private CANdle candle;

  // LED modes
  public RainbowAnimation rainbowAnim = new RainbowAnimation(.5, .89, 64);
  public LarsonAnimation larsonAnim = new LarsonAnimation(177, 156, 217);
  public TwinkleAnimation twinkleAnim = new TwinkleAnimation(204, 100, 255);

  public ElevatorLight() {
    candle = new CANdle(Hardware.ELEVATOR_LED);
    configureCandle();
    // candle.setLEDs(28, 3, 3);
    // candle.animate(larsonAnim);
    // candle.animate(rainbowAnim);
    candle.animate(twinkleAnim);
  }

  private void configureCandle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    candle.configAllSettings(config);
  }

  public Command animate(Animation animation) {
    return runOnce(() -> candle.animate(animation));
  }
}
