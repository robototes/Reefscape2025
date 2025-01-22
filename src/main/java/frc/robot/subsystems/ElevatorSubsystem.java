package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final double ELEVATOR_KP = 0.1;
  private final double ELEVATOR_KI = 0;
  private final double ELEVATOR_KD = 0;
  private final double ELEVATOR_KS = 0;
  private final double ELEVATOR_KV = 0;
  private final double ELEVATOR_KG = 0;
  private final double ELEVATOR_KA = 0;
  private final double reverseSoftLimit = -67;
  private final double forwardSoftLimit = -1;

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD, new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

  private TalonFX m_motor;
  private TalonFX m_motor2;

  private final MutVoltage m_appliedVoltage = Units.Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Units.Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = Units.RadiansPerSecond.mutable(0);
  // Creates a SysIdRoutine
  SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));

  /** Subsystem constructor. */
  public ElevatorSubsystem() {
    // m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    m_motor = new TalonFX(40);
    var talonFXConfigurator = m_motor.getConfigurator();
    m_motor2 = new TalonFX(41);
    var talonFXConfigurator2 = m_motor2.getConfigurator();
    var currentLimits = new CurrentLimitsConfigs();
    var softLimits = new SoftwareLimitSwitchConfigs();
    // soft limits
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = forwardSoftLimit;
    softLimits.ReverseSoftLimitThreshold = reverseSoftLimit;
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
    // enable stator current limit
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

  public void reachGoal(double goal) {
    m_controller.setGoal(goal);
    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_motor.getPosition().getValueAsDouble());
    double feedforwardOutput =
        0; // m_feedforward.calculate(m_controller.getSetpoint().velocity); can be used later to
    // calculate with correct values
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  public Command goUp() {
    return startEnd(
        () -> m_motor.set(-0.25), () -> m_motor.set(-0.02)); // test values from dev model bot
  }

  public Command goDown() {
    return startEnd(
        () -> m_motor.set(0.04), () -> m_motor.set(-0.02)); // test values from dev model bot
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    // m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }
}
