package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import java.util.function.Supplier;

public class SpinnyClaw extends SubsystemBase {
  public static final double INTAKE_SPEED = -4;
  public static final double EXTAKE_SPEED = 2;
  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX motor;

  //alerts
  private final Alert NotConnectedError = new Alert("Spinny Claw", "Motor not connected", AlertType.kError);

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

  public Command intakePower() {
    return runOnce(() -> motor.setVoltage(INTAKE_SPEED)).withName("Intake power");
  }

  public Command extakePower() {
    return runOnce(() -> motor.setVoltage(EXTAKE_SPEED)).withName("Extake power");
  }

  public Command holdIntakePower() {
    return startEnd(() -> motor.setVoltage(INTAKE_SPEED), () -> motor.stopMotor())
        .withName("Hold intake power");
  }

  public Command holdExtakePower() {
    return startEnd(() -> motor.setVoltage(EXTAKE_SPEED), () -> motor.stopMotor())
        .withName("Hold extake power");
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName("Spinny claw stop");
  }

  public void periodic() {
    NotConnectedError.set(!motor.getMotorVoltage().hasUpdated());
  }
}
