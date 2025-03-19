package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Hardware;
import java.util.function.Supplier;

public class groundPivot extends SubsystemBase {
  // Presets
  private final double groundPivot_KP = 20;
  private final double groundPivot_KI = 0;
  private final double groundPivot_KD = 0;
  private final double groundPivot_KS = 0.1;
  private final double groundPivot_KV = 0.69;
  private final double groundPivot_KG = 0.18;
  private final double groundPivot_KA = 0.0;

  public static final double HARDSTOP_HIGH = 0.32;
  public static final double HARDSTOP_LOW = -0.26;

  // Constant for gear ratio (the power that one motor gives to gear)
  private static final double ARM_RATIO = (16 / 32) * (16 / 32);
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // TalonFX
  private final TalonFX motor;

  // alerts
  private final Alert NotConnectedError =
      new Alert("groundPivot", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  private final SysIdRoutine routine;

  private double targetPos;

  public groundPivot() {
    motor = new TalonFX(Hardware.GROUND_PIVOT_MOTOR_ID);
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).div(Seconds.of(1)), Volts.of(1), Seconds.of(2)),
            new SysIdRoutine.Mechanism(
                (voltage) -> motor.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("groundPivotMotor")
                        .voltage(motor.getMotorVoltage().getValue())
                        .angularPosition(motor.getPosition().getValue())
                        .angularVelocity(motor.getVelocity().getValue()),
                this));
    factoryDefaults();
    logTabs();
  }

  // commands
  public Command SysIDQuasistatic(Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command SysIDDynamic(Direction direction) {
    return routine.dynamic(direction);
  }

  private Command setTargetPosition(double pos) {
    return runOnce(
        () -> {
          motor.setControl(m_request.withPosition(pos));
          targetPos = pos;
        });
  }

  public boolean atPosition(double position) {
    return MathUtil.isNear(position, getCurrentPosition(), POS_TOLERANCE);
  }

  private double getTargetPosition() {
    return targetPos;
  }

  private double getCurrentPosition() {
    var curPos = motor.getPosition();
    return curPos.getValueAsDouble();
  }

  // preset command placeholder
  public Command moveToPosition(double position) {
    return setTargetPosition(position)
        .andThen(
            Commands.waitUntil(() -> Math.abs(getCurrentPosition() - position) < POS_TOLERANCE));
  }

  // (+) is to move arm up, and (-) is down
  public Command startMovingVoltage(Supplier<Voltage> speedControl) {
    return runEnd(() -> motor.setVoltage(speedControl.get().in(Volts)), () -> motor.stopMotor());
  }

  // logging
  public void logTabs() {
    Shuffleboard.getTab("ground pivot")
        .addDouble("Pivot Speed", () -> motor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("ground pivot")
        .addDouble("Pivot Motor Temperature", () -> motor.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("ground pivot").addDouble("Pivot Position", () -> getCurrentPosition());
    Shuffleboard.getTab("ground pivot").addDouble("Pivot Target Pos", () -> getTargetPosition());
    Shuffleboard.getTab("ground pivot")
        .addDouble("Pivot Motor rotor Pos", () -> motor.getRotorPosition().getValueAsDouble());
  }

  // TalonFX config
  public void factoryDefaults() {
    TalonFXConfigurator cfg = motor.getConfigurator();
    var talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = Hardware.GROUND_PIVOT_ENCODER_ID;
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANCoder;
    talonFXConfiguration.Feedback.RotorToSensorRatio = 1 / ARM_RATIO;

    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 20; // starting low for testing
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 10; // starting low for testing
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // PID
    // set slot 0 gains
    talonFXConfiguration.Slot0.kS = groundPivot_KS;
    talonFXConfiguration.Slot0.kV = groundPivot_KV;
    talonFXConfiguration.Slot0.kA = groundPivot_KA;
    talonFXConfiguration.Slot0.kP = groundPivot_KP;
    talonFXConfiguration.Slot0.kI = groundPivot_KI;
    talonFXConfiguration.Slot0.kD = groundPivot_KD;
    talonFXConfiguration.Slot0.kG = groundPivot_KG;
    talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // set Motion Magic settings in rps not mechanism units
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        80; // Target cruise velocity of 80 rps
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    talonFXConfiguration.MotionMagic.MotionMagicJerk =
        1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    cfg.apply(talonFXConfiguration);
  }

  // alert
  @Override
  public void periodic() {
    NotConnectedError.set(notConnectedDebouncer.calculate(!motor.getMotorVoltage().hasUpdated()));
  }
}