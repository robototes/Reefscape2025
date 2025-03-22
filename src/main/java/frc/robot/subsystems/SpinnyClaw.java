package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.sensors.ArmSensor;
import java.util.function.Supplier;

public class SpinnyClaw extends SubsystemBase {
  public static final double CORAL_INTAKE_SPEED = -6;
  public static final double CORAL_EXTAKE_SPEED = 4;
  public static final double CORAL_L4_EXTAKE_SPEED = 2;
  public static final double ALGAE_INTAKE_SPEED = -3;
  public static final double ALGAE_GRIP_INTAKE_SPEED = -2.5;
  public static final double ALGAE_EXTAKE_SPEED = 12; // untested
  public static final double ALGAE_PROCESSOR_EXTAKE_SPEED = 8; // untested
  public static final double ALGAE_FLING_SPEED = 10;
  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX motor;
  private final ArmSensor armSensor;

  // alerts
  private final Alert NotConnectedError =
      new Alert("Spinny Claw", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  public SpinnyClaw(ArmSensor armSensor) {
    motor = new TalonFX(Hardware.SPINNY_CLAW_MOTOR_ID);
    this.armSensor = armSensor;
    configMotors();
    logTabs();
  }

  // (+) is to intake out, and (-) is in
  public Command movingVoltage(Supplier<Voltage> speedControl) {
    return run(() -> motor.setVoltage(speedControl.get().in(Volts)))
        .finallyDo(() -> motor.setVoltage(0))
        .withName("Spinny claw moving voltage");
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
    if (armSensor != null) {
      return runOnce(
          () -> {
            // if (armSensor.booleanInClaw() && pow < 0) {
            //   motor.stopMotor();
            // } else {
            motor.setVoltage(pow);
            // }
          });
    } else {
      return runOnce(() -> motor.setVoltage(pow));
    }
  }

  private Command holdPower(double pow) {
    if (armSensor != null) {
      return startEnd(
          () -> {
            // if (armSensor.booleanInClaw() && pow < 0) {
            //   motor.stopMotor();
            // } else {
            motor.setVoltage(pow);
            // }
          },
          () -> motor.stopMotor());
    } else {
      return startEnd(() -> motor.setVoltage(pow), () -> motor.stopMotor());
    }
  }

  public Command coralIntakePower() {
    return setPower(CORAL_INTAKE_SPEED).withName("Intake power");
  }

  public Command coralExtakePower() {
    return setPower(CORAL_EXTAKE_SPEED).withName("Extake power");
  }

  public Command coralHoldIntakePower() {
    return holdPower(CORAL_INTAKE_SPEED).withName("Hold intake power");
  }

  public Command coralHoldExtakePower() {
    return holdPower(CORAL_EXTAKE_SPEED).withName("Hold extake power");
  }

  // algae stuff
  public Command algaeIntakePower() {
    return setPower(ALGAE_INTAKE_SPEED).withName("Algae intake power");
  }

  public Command
      algaeExtakePower() { // can change to net extake maybe? also bound as general extake though
    return setPower(ALGAE_EXTAKE_SPEED).withName("Algae extake power");
  }

  public Command algaeGripIntakePower() {
    return setPower(ALGAE_GRIP_INTAKE_SPEED).withName("Algae grip intake power");
  }

  public Command algaeHoldExtakePower() {
    return holdPower(ALGAE_EXTAKE_SPEED).withName("Algae hold extake power");
  }

  public Command algaeExtakeProcessorPower() {
    return setPower(ALGAE_PROCESSOR_EXTAKE_SPEED).withName("Algae processor extake power");
  }

  public Command algaeFlingPower() {
    return setPower(ALGAE_FLING_SPEED).withName("Algae fling power");
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName("Spinny claw stop");
  }

  @Override
  public void periodic() {
    NotConnectedError.set(notConnectedDebouncer.calculate(!motor.getMotorVoltage().hasUpdated()));
  }
}
