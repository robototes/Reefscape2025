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
  public static final double CORAL_INTAKE_SPEED = -4;
  public static final double CORAL_EXTAKE_SPEED = 2;
  public static final double ALGAE_INTAKE_SPEED = -2; // untested
  public static final double ALGAE_EXTAKE_SPEED = 2; // untested
  public static final double ALGAE_FLING_SPEED = 2; // untested
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
    Shuffleboard.getTab("Claw")
        .addDouble("Claw Speed", () -> motor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("Claw")
        .addDouble("Claw Motor Temperature", () -> motor.getDeviceTemp().getValueAsDouble());
  }

  // TalonFX config
  public void configMotors() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    TalonFXConfigurator cfg = motor.getConfigurator();
    var currentLimits = new CurrentLimitsConfigs();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.apply(configuration);
    // enabling stator current limits
    currentLimits.StatorCurrentLimit = 150; // subject to change
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 40; // subject to change
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);
  }

  private Command setPower(double pow) {
    return runOnce(() -> motor.setVoltage(pow));
  }

  private Command holdPower(double pow) {
    return startEnd(() -> motor.setVoltage(pow), () -> motor.stopMotor());
  }

  public Command intakePower() {
    return setPower(CORAL_INTAKE_SPEED).withName("Intake power");
  }

  public Command extakePower() {
    return setPower(CORAL_EXTAKE_SPEED).withName("Extake power");
  }

  public Command holdIntakePower() {
    return holdPower(CORAL_INTAKE_SPEED).withName("Hold intake power");
  }

  public Command holdExtakePower() {
    return holdPower(CORAL_EXTAKE_SPEED).withName("Hold extake power");
  }

  // algae stuff
  public Command algaeIntakePower() {
    return setPower(ALGAE_INTAKE_SPEED).withName("Algae Intake power");
  }

  public Command algaeExtakePower() {
    return setPower(ALGAE_EXTAKE_SPEED).withName("Algae Extake power");
  }

  public Command algaeHoldIntakePower() { // tbh idk if it works
    return holdPower(ALGAE_INTAKE_SPEED).withName("Algae hold intake power");
  }

  public Command algaeHoldExtakePower() { // tbh idk if it works
    return holdPower(ALGAE_EXTAKE_SPEED).withName("Algae hold extake power");
  }

  public Command algaeFlingPower() {
    return setPower(ALGAE_FLING_SPEED).withName("Algae fling power");
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName("Spinny claw stop");
  }
}
