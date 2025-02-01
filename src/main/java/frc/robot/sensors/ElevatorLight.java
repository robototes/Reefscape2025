package frc.robot.sensors;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ElevatorLight extends SubsystemBase {

  private CANdle candle;

  // LED modes
  public RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);

  public ElevatorLight() {
    candle = new CANdle(Hardware.ELEVATOR_LED);
    configureCandle();
    candle.setLEDs(20, 28, 13);
  }

  private void configureCandle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    candle.configAllSettings(config);
  }

  public Command animate(Animation animation) {
    return run(() -> candle.animate(animation));
  }
}
