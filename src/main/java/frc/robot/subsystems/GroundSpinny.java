package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;

public class GroundSpinny extends SubsystemBase {
  public static final double GROUND_INTAKE_SPEED = -8;
  public static final double GROUND_INTAKE_JITTER_SPEED = 1;
  public static final double FUNNEL_INTAKE_SPEED = -2;
  public static final double QUICK_HANDOFF_EXTAKE_SPEED = 1;
  private static final double STATOR_CURRENT_STALL_THRESHOLD = 50;

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
    currentLimits.StatorCurrentLimit = 100; // subject to change
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 20; // subject to change
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);
  }

  private Command setPower(double pow) {
    return runOnce(
        () -> {
          motor.setVoltage(pow);
          lastSetPower = pow;
        });
  }

  private Command holdPower(double pow) {
    return startEnd(
        () -> {
          motor.setVoltage(pow);
          lastSetPower = pow;
        },
        () -> motor.stopMotor());
  }

  public Command setFunnelIntakePower() {
    return setPower(FUNNEL_INTAKE_SPEED).withName("set funnel intake power");
  }

  public Command holdFunnelIntakePower() {
    return holdPower(FUNNEL_INTAKE_SPEED).withName("hold funnel intake power");
  }

  public void imperativeSetGroundIntakePower() {
    motor.setVoltage(GROUND_INTAKE_SPEED);
    lastSetPower = GROUND_INTAKE_SPEED;
  }

  public Command setGroundIntakePower() {
    return setPower(GROUND_INTAKE_SPEED).withName("set ground intake power");
  }

  public Command holdGroundIntakePower() {
    return holdPower(GROUND_INTAKE_SPEED).withName("hold ground intake power");
  }

  public Command setQuickHandoffExtakeSpeed() {
    return setPower(QUICK_HANDOFF_EXTAKE_SPEED).withName("set quick handoff extake power");
  }

  public Command holdQuickHandoffExtakeSpeed() {
    return holdPower(QUICK_HANDOFF_EXTAKE_SPEED).withName("hold quick handoff extake power");
  }

  public Command setGroundIntakeJitterSpeed() {
    return setPower(GROUND_INTAKE_JITTER_SPEED).withName("set ground intake jitter power");
  }

  public Command holdGroundIntakeJitterSpeed() {
    return holdPower(GROUND_INTAKE_JITTER_SPEED).withName("hold ground intake jitter power");
  }

  public Trigger stalling() {
    return new Trigger(
        () -> motor.getStatorCurrent().getValueAsDouble() > STATOR_CURRENT_STALL_THRESHOLD);
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName("Ground spinny stop");
  }
}
