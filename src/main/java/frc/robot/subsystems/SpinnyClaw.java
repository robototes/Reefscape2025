package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import java.util.function.Supplier;

public class SpinnyClaw extends SubsystemBase {
  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX motor;

  public SpinnyClaw() {
    motor = new TalonFX(Hardware.SPINNY_CLAW_MOTOR_ID);
    configMotors();
    logTabs();
  }

  // (+) is to move arm up, and (-) is down
  public Command movingVoltage(Supplier<Voltage> speedControl) {
    return run(() -> motor.setVoltage(speedControl.get().in(Volts)))
        .finallyDo(() -> motor.setVoltage(0));
  }

  // Log tabs to shuffleboard, temperature, and motor speed
  public void logTabs() {
    Shuffleboard.getTab("claw-info")
        .addDouble("claw_speed", () -> motor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("claw-info")
        .addDouble("claw_motor_temp", () -> motor.getDeviceTemp().getValueAsDouble());
  }

  // TalonFX config
  public void configMotors() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    TalonFXConfigurator cfg = motor.getConfigurator();
    var currentLimits = new CurrentLimitsConfigs();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.apply(configuration);
    // enabling stator current limits
    currentLimits.StatorCurrentLimit = 5; // starting low for testing
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 5; // starting low for testing
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);
  }
}
