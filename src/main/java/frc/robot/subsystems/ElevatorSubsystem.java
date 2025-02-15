package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Hardware;

public class ElevatorSubsystem extends SubsystemBase {
  public static final double LEVEL_FOUR_POS = 8;
  public static final double LEVEL_THREE_POS = 6;
  public static final double LEVEL_TWO_POS = 4;
  public static final double LEVEL_ONE_POS = 2;
  public static final double STOWED = 1;
  public static final double INTAKE = 0;
  public static final double MANUAL = 1;
  private static final double POS_TOLERANCE = 0.02;
  private final double ELEVATOR_KP = 3; // add feedfwds for each stage?
  private final double ELEVATOR_KI = 0;
  private final double ELEVATOR_KD = 0;
  private final double ELEVATOR_KS = 0;
  private final double ELEVATOR_KV = 0;
  private final double ELEVATOR_KA = 0;
  private final double REVERSE_SOFT_LIMIT = INTAKE - 0.05;
  private final double FORWARD_SOFT_LIMIT = LEVEL_FOUR_POS + 1;
  private final double UP_VOLTAGE = -3;
  private final double DOWN_VOLTAGE = 3;
  private final double HOLD_VOLTAGE = 0;
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // motors
  private TalonFX m_motor;
  private TalonFX m_motor2;

  private double curPos;
  private double targetPos;
  private boolean hasBeen0ed;

  private final MutVoltage m_appliedVoltage = Units.Volts.mutable(0);
  // Creates a SysIdRoutine
  SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

  /** Subsystem constructor. */
  public ElevatorSubsystem() {
    // m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    m_motor = new TalonFX(Hardware.ELEVATOR_MOTOR_ONE);
    m_motor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_TWO);
    motorConfigs();
    m_motor2.setControl(new Follower(m_motor.getDeviceID(), true));
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    // SmartDashboard.putData("Elevator Sim", m_mech2d);
    Shuffleboard.getTab("Elevator").addDouble("Motor Current Position", () -> getCurrentPosition());
    Shuffleboard.getTab("Elevator").addDouble("Target Position", () -> getTargetPosition());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public void voltageDrive(Voltage drive) {
    m_motor.setVoltage(drive.in(Units.Volts));
    m_motor2.setVoltage(drive.in(Units.Volts));
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
    var currentLimits = new CurrentLimitsConfigs();
    var softLimits = new SoftwareLimitSwitchConfigs();
    // soft limits
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;
    softLimits.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT;
    talonFXConfigurator.apply(softLimits);
    talonFXConfigurator2.apply(softLimits);
    // enable stator current limit
    currentLimits.StatorCurrentLimit = 10;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 10;
    currentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigurator.apply(currentLimits);
    talonFXConfigurator2.apply(currentLimits);
    // create brake mode for motors
    var outputConfigs = new MotorOutputConfigs();
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);
    talonFXConfigurator2.apply(outputConfigs);

    // set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ELEVATOR_KS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ELEVATOR_KV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ELEVATOR_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ELEVATOR_KP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = ELEVATOR_KI; // no output for integrated error
    slot0Configs.kD = ELEVATOR_KD; // A velocity error of 1 rps results in 0.1 V output
    talonFXConfigurator.apply(slot0Configs);

    // set Motion Magic settings
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    talonFXConfigurator.apply(motionMagicConfigs);
  }

  private Command setTargetPosition(double pos) {
    return runOnce(
        () -> {
          if (hasBeen0ed) {
            m_motor.setControl(m_request.withPosition(pos));
            targetPos = pos;
          }
        });
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
        });
  }

  public Command setLevel(double pos) {
    return setTargetPosition(pos)
        .until(() -> Math.abs(getCurrentPosition() - pos) < POS_TOLERANCE)
        .withName("setLevel" + pos);
  }

  public Command goUp() {
    return defer(() -> setLevel(getCurrentPosition() + MANUAL));
  }

  public Command goDown() {
    return defer(() -> setLevel(getCurrentPosition() - MANUAL));
  }

  public Command goUpPower() {
    return startEnd(() -> m_motor.setVoltage(UP_VOLTAGE), () -> m_motor.setVoltage(HOLD_VOLTAGE));
  }

  public Command goDownPower() {
    return startEnd(() -> m_motor.setVoltage(DOWN_VOLTAGE), () -> m_motor.setVoltage(HOLD_VOLTAGE));
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    // m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }
}
