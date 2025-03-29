package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class GroundSpinny extends SubsystemBase {
  public static final double GROUND_INTAKE_SPEED = -2;

  // TalonFX
  private final TalonFX motor;

  // logs
  private double lastSetPower;

  public void logTabs() {
    Shuffleboard.getTab("GroundIntake")
        .addDouble("Speed", () -> motor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("GroundIntake")
        .addDouble("Motor Temperature", () -> motor.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("GroundIntake")
        .addDouble("Motor Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
    Shuffleboard.getTab("GroundIntake").addDouble("Last Set Power", () -> lastSetPower);
  }

  public GroundSpinny() {
    motor = new TalonFX(Hardware.GROUND_INTAKE_SPINNY_MOTOR);
    configMotors();
    logTabs();
  }

  // TalonFX config
  public void configMotors() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    TalonFXConfigurator cfg = motor.getConfigurator();
    var currentLimits = new CurrentLimitsConfigs();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.apply(configuration);
    // enabling stator current limits
    currentLimits.StatorCurrentLimit = 10; // subject to change
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 10; // subject to change
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);
  }

  private Command holdPower(double pow) {
    return startEnd(
        () -> {
          motor.setVoltage(pow);
          lastSetPower = pow;
        },
        () -> motor.stopMotor());
  }

  public Command holdIntakePower() {
    return holdPower(GROUND_INTAKE_SPEED).withName("holdIntakePower");
  }
}
