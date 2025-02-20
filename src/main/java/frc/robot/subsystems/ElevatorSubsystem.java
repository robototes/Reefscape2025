package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Hardware;
import java.util.function.DoubleConsumer;

public class ElevatorSubsystem extends SubsystemBase {
  // Maximum is 38.34
  public static final double LEVEL_FOUR_PRE_POS = 37.5;
  public static final double LEVEL_FOUR_POS = 36.5;
  public static final double LEVEL_THREE_POS = 13;
  public static final double LEVEL_TWO_POS = 3.15;
  public static final double LEVEL_ONE_POS = 8.06;
  public static final double STOWED = 2;
  public static final double INTAKE = 0.1;
  public static final double PRE_INTAKE = 2;
  public static final double MANUAL = 0.1;
  private static final double POS_TOLERANCE = 0.1;
  private final double ELEVATOR_KP = 13.804; // add feedfwds for each stage?
  private final double ELEVATOR_KI = 0;
  private final double ELEVATOR_KD = 0.079221;
  private final double ELEVATOR_KS = 0.33878;
  private final double ELEVATOR_KV = 0.12975;
  private final double ELEVATOR_KA = 0.0070325;
  private final double REVERSE_SOFT_LIMIT = -0.05;
  private final double FORWARD_SOFT_LIMIT = 38;
  private final double UP_VOLTAGE = 5;
  private final double DOWN_VOLTAGE = -3;
  private final double HOLD_VOLTAGE = 0.6;
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // motors
  private TalonFX m_motor;
  private TalonFX m_motor2;

  private double curPos;
  private double targetPos;
  private boolean hasBeen0ed;

  private DoubleConsumer rumble = (rumble) -> {};

  private final MutVoltage m_appliedVoltage = Units.Volts.mutable(0);
  // Creates a SysIdRoutine
  SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

  /** Subsystem constructor. */
  public ElevatorSubsystem() {
    // m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    m_motor = new TalonFX(Hardware.ELEVATOR_MOTOR_ONE, "Drivebase");
    m_motor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_TWO, "Drivebase");
    motorConfigs();
    m_motor2.setControl(new Follower(m_motor.getDeviceID(), true));

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    // SmartDashboard.putData("Elevator Sim", m_mech2d);
    Shuffleboard.getTab("Elevator").addDouble("Motor Current Position", () -> getCurrentPosition());
    Shuffleboard.getTab("Elevator").addDouble("Target Position", () -> getTargetPosition());
    Shuffleboard.getTab("Elevator")
        .addDouble("M1 supply current", () -> m_motor.getSupplyCurrent().getValueAsDouble());
    Shuffleboard.getTab("Elevator")
        .addDouble("M2 supply current", () -> m_motor2.getSupplyCurrent().getValueAsDouble());
    Shuffleboard.getTab("Elevator").addBoolean("Is zero'd", () -> getHasBeen0ed());
    Shuffleboard.getTab("Elevator")
        .addDouble("M1 temp", () -> m_motor.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("Elevator")
        .addDouble("M2 temp", () -> m_motor2.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("Elevator")
        .addDouble("M1 output voltage", () -> m_motor.getMotorVoltage().getValueAsDouble());
    Shuffleboard.getTab("Elevator")
        .addDouble("M2 output voltage", () -> m_motor2.getMotorVoltage().getValueAsDouble());
    Shuffleboard.getTab("Elevator")
        .addBoolean("M1 at forward softstop", () -> m_motor.getFault_ForwardSoftLimit().getValue());
    Shuffleboard.getTab("Elevator")
        .addBoolean("M1 at reverse softstop", () -> m_motor.getFault_ReverseSoftLimit().getValue());
    Shuffleboard.getTab("Elevator")
        .addBoolean(
            "M2 at forward softstop", () -> m_motor2.getFault_ForwardSoftLimit().getValue());
    Shuffleboard.getTab("Elevator")
        .addBoolean(
            "M2 at reverse softstop", () -> m_motor2.getFault_ReverseSoftLimit().getValue());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public void voltageDrive(Voltage drive) {
    m_motor.setVoltage(drive.in(Units.Volts));
    m_motor2.setVoltage(-drive.in(Units.Volts));
  }

  public void logMotors(SysIdRoutineLog log) { // in theory this should work?
    log.motor("elevator-motor")
        .voltage(
            m_appliedVoltage.mut_replace(
                m_motor.get() * RobotController.getBatteryVoltage(), Units.Volts))
        .angularPosition(m_motor.getPosition().getValue())
        .angularVelocity(m_motor.getVelocity().getValue());
    log.motor("elevator-motor2")
        .voltage(
            m_appliedVoltage.mut_replace(
                m_motor2.get() * RobotController.getBatteryVoltage(), Units.Volts))
        .angularPosition(m_motor2.getPosition().getValue())
        .angularVelocity(m_motor2.getVelocity().getValue());
  }

  public void motorConfigs() {
    // add FOC at some point please --gives more torque
    var talonFXConfigurator = m_motor.getConfigurator();
    var talonFXConfigurator2 = m_motor2.getConfigurator();
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    // enable stator current limit
    configuration.CurrentLimits.StatorCurrentLimit = 160;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = 80;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // create brake mode for motors
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motor 2 gets current limits and motor output mode, but not softlimits or PID
    talonFXConfigurator2.apply(configuration);

    // soft limits
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT;

    // set slot 0 gains
    configuration.Slot0.kS = ELEVATOR_KS; // Add 0.25 V output to overcome static friction
    configuration.Slot0.kV = ELEVATOR_KV; // A velocity target of 1 rps results in 0.12 V output
    configuration.Slot0.kA = ELEVATOR_KA; // An acceleration of 1 rps/s requires 0.01 V output
    configuration.Slot0.kP =
        ELEVATOR_KP; // A position error of 2.5 rotations results in 12 V output
    configuration.Slot0.kI = ELEVATOR_KI; // no output for integrated error
    configuration.Slot0.kD = ELEVATOR_KD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    // Bottom to full: ~40 rotations
    // Constant jerk:
    //   x = 1/6 j t^3
    //   j = 6 x / t^3
    // Considering half of the movement:
    //   j = 6 (x/2) / (t/2)^3
    //     = 24 x / t^3
    // Maximum acceleration:
    //   a = j (t/2)
    //     = (24 x / t^3) (t/2)
    //     = 12 x / t^2
    // For full travel in 0.8 seconds:
    //   j = 24 (40 rot) / (0.8 s)^3
    //     = 1875 rot/s^3
    //   a = 12 (40 rot) / (0.8 s)^2
    //     = 750 rot/s^2
    // For full travel in 0.75 seconds:
    //   j = 24 (40 rot) / (0.75 s)^3
    //     = (61440 / 27) rot/s^3
    //     = 2275.56 rot/s^3
    //   a = 12 (40 rot) / (0.75 s)^2
    //     = (7680 / 9) rot/s^2
    //     = 853.33 rot/s^2
    // MotionMagic uses mechanism rotations per second*
    configuration.MotionMagic.MotionMagicCruiseVelocity = 80;
    configuration.MotionMagic.MotionMagicAcceleration = 750;
    configuration.MotionMagic.MotionMagicJerk = 1875;

    talonFXConfigurator.apply(configuration);
  }

  public void setRumble(DoubleConsumer rumble) {
    this.rumble = rumble;
  }

  public boolean getHasBeen0ed() {
    return hasBeen0ed;
  }

  private double getTargetPosition() {
    return targetPos;
  }

  private double getCurrentPosition() {
    curPos = m_motor.getPosition().getValueAsDouble();
    return curPos;
  }

  private void setCurrentPosition(double pos) {
    m_motor.setPosition(pos);
  }

  public Command resetPosZero() {
    return runOnce(
        () -> {
          setCurrentPosition(0);
          hasBeen0ed = true;
          rumble.accept(0);
        });
  }

  public Command setLevel(double pos) {
    return runOnce(
            () -> {
              if (hasBeen0ed) {
                m_motor.setControl(m_request.withPosition(pos));
                m_motor2.setControl(new Follower(m_motor.getDeviceID(), true));
                targetPos = pos;
              } else {
                rumble.accept(0.2);
              }
            })
        .andThen(Commands.waitUntil(() -> Math.abs(getCurrentPosition() - pos) < POS_TOLERANCE))
        .withName("setLevel" + pos);
  }

  public Command goUp() {
    return defer(() -> setLevel(getCurrentPosition() + MANUAL));
  }

  public Command goDown() {
    return defer(() -> setLevel(getCurrentPosition() - MANUAL));
  }

  public Command goUpPower() {
    return startEnd(
            () -> {
              m_motor.setVoltage(UP_VOLTAGE);
              m_motor2.setVoltage(-UP_VOLTAGE);
            },
            () -> {
              m_motor.stopMotor();
              m_motor2.stopMotor();
              // m_motor.setVoltage(HOLD_VOLTAGE);
              // m_motor2.setVoltage(-HOLD_VOLTAGE);
            })
        .withName("Elevator up power");
  }

  public Command goDownPower() {
    return startEnd(
            () -> {
              m_motor.setVoltage(DOWN_VOLTAGE);
              m_motor2.setVoltage(-DOWN_VOLTAGE);
            },
            () -> {
              m_motor.stopMotor();
              m_motor2.stopMotor();
              // m_motor.setVoltage(HOLD_VOLTAGE);
              // m_motor2.setVoltage(-HOLD_VOLTAGE);
            })
        .withName("Elevator down power");
  }

  /** Stop the control loop and motor output. */
  public Command stop() {
    return runOnce(() -> m_motor.stopMotor()).ignoringDisable(true).withName("ElevatorStop");
  }
}
