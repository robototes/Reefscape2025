package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {
  public static final double LEVEL_FOUR_POS = 4;
  public static final double LEVEL_THREE_POS = 3;
  public static final double LEVEL_TWO_POS = 2;
  public static final double LEVEL_ONE_POS = 1;
  private static final double POS_TOLERANCE = 0.1;
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final double ELEVATOR_KP = 0.1;
  private final double ELEVATOR_KI = 0;
  private final double ELEVATOR_KD = 0;
  private final double ELEVATOR_KS = 0;
  private final double ELEVATOR_KV = 0;
  private final double ELEVATOR_KA = 0;
  private final double REVERSE_SOFT_LIMIT = -67;
  private final double FORWARD_SOFT_LIMIT = -1;
  private final double UP_VOLTAGE = -0.25;
  private final double DOWN_VOLTAGE = 0.04;
  private final double HOLD_VOLTAGE = -0.02;
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // motors
  private TalonFX m_motor;
  private TalonFX m_motor2;

  private final MutVoltage m_appliedVoltage = Units.Volts.mutable(0);
  // Creates a SysIdRoutine
  SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

  /** Subsystem constructor. */
  public ElevatorSubsystem() {
    // m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    m_motor = new TalonFX(40);
    m_motor2 = new TalonFX(41);
    motorConfigs();
    m_motor2.setControl(new Follower(m_motor.getDeviceID(), false));
    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    // SmartDashboard.putData("Elevator Sim", m_mech2d);
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
    currentLimits.StatorCurrentLimit = 5; // starting low for testing
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 5; // starting low for testing
    currentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigurator.apply(currentLimits);
    talonFXConfigurator2.apply(currentLimits);
    // create brake mode for motors
    var outputConfigs = new MotorOutputConfigs();
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);
    talonFXConfigurator2.apply(outputConfigs);

    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = ELEVATOR_KS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ELEVATOR_KV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ELEVATOR_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ELEVATOR_KP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = ELEVATOR_KI; // no output for integrated error
    slot0Configs.kD = ELEVATOR_KD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_motor.getConfigurator().apply(talonFXConfigs);
  }

  private Command setTargetPosition(double pos) {
    // set target position to 100 rotations
    return runOnce(() -> m_motor.setControl(m_request.withPosition(pos)));
  }

  private double getCurrentPosition() {
    var curPos = m_motor.getPosition();
    return curPos.getValueAsDouble();
  }

  public Command setLevel(double pos) {
    return setTargetPosition(pos).until(() -> Math.abs(getCurrentPosition() - pos) < POS_TOLERANCE);
  }

  public Command goUp() {
    return startEnd(
        () -> m_motor.set(UP_VOLTAGE),
        () -> m_motor.set(HOLD_VOLTAGE)); // test values from dev model bot
  }

  public Command goDown() {
    return startEnd(
        () -> m_motor.set(DOWN_VOLTAGE),
        () -> m_motor.set(HOLD_VOLTAGE)); // test values from dev model bot
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    // m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }
}
