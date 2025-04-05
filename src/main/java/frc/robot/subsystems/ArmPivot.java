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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Hardware;
import java.util.function.Supplier;

public class ArmPivot extends SubsystemBase {
  // Presets
  private final double ARMPIVOT_KP = 20;
  private final double ARMPIVOT_KI = 0;
  private final double ARMPIVOT_KD = 0;
  private final double ARMPIVOT_KS = 0.1;
  private final double ARMPIVOT_KV = 0.69;
  private final double ARMPIVOT_KG = 0.18;
  private final double ARMPIVOT_KA = 0.0;
  public static final double CORAL_PRESET_L1 = 0;
  public static final double CORAL_PRESET_L2 = 0.13;
  public static final double CORAL_PRESET_L3 = 0.13;
  public static final double CORAL_PRESET_L4 = 0.0;
  public static final double CORAL_PRESET_PRE_L4 = 1.0 / 16.0;
  public static final double CORAL_POST_SCORE = -0.15;
  public static final double ALGAE_REMOVE_PREPOS = 0;
  public static final double ALGAE_REMOVE = 0;
  public static final double ALGAE_FLING = -0.08;
  public static final double ALGAE_STOWED = -0.05;
  public static final double ALGAE_PROCESSOR_SCORE = -0.05;
  public static final double ALGAE_GROUND_INTAKE = -0.085;
  public static final double ALGAE_NET_SCORE = 0.175; // untested - old value was 0.18
  public static final double CORAL_PRESET_STOWED = 0.125;
  public static final double CORAL_PRESET_OUT = 0;
  public static final double CORAL_PRESET_UP = 0.25; // Pointing directly upwards
  public static final double CORAL_PRESET_GROUND_INTAKE = 0.25; // untested - using preset up pos
  public static final double CORAL_PRESET_DOWN = -0.25;
  public static final double HARDSTOP_HIGH = 0.32;
  public static final double HARDSTOP_LOW = -0.26;
  public static final double POS_TOLERANCE = Units.degreesToRotations(5);
  public static final double PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
  // Constant for gear ratio (the power that one motor gives to gear)
  private static final double ARM_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (18.0 / 48.0);

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // TalonFX
  private final TalonFX motor;

  // alerts
  private final Alert NotConnectedError =
      new Alert("ArmPivot", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  private final SysIdRoutine routine;

  private double targetPos;

  public ArmPivot() {
    motor = new TalonFX(Hardware.ARM_PIVOT_MOTOR_ID);
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).div(Seconds.of(1)), Volts.of(1), Seconds.of(2)),
            new SysIdRoutine.Mechanism(
                (voltage) -> motor.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("armPivotMotor")
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
    return setTargetPosition(position).andThen(Commands.waitUntil(atAngle(position)));
  }

  public Trigger atAngle(double position) {
    return new Trigger(() -> Math.abs(getCurrentPosition() - position) < POS_TOLERANCE);
  }

  // (+) is to move arm up, and (-) is down
  public Command startMovingVoltage(Supplier<Voltage> speedControl) {
    return runEnd(() -> motor.setVoltage(speedControl.get().in(Volts)), () -> motor.stopMotor());
  }

  // logging
  public void logTabs() {
    Shuffleboard.getTab("Arm Pivot")
        .addDouble("Pivot Speed", () -> motor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("Arm Pivot")
        .addDouble("Pivot Motor Temperature", () -> motor.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("Arm Pivot").addDouble("Pivot Position", () -> getCurrentPosition());
    Shuffleboard.getTab("Arm Pivot").addDouble("Pivot Target Pos", () -> getTargetPosition());
    Shuffleboard.getTab("Arm Pivot")
        .addDouble("Pivot Motor rotor Pos", () -> motor.getRotorPosition().getValueAsDouble());
  }

  // TalonFX config
  public void factoryDefaults() {
    TalonFXConfigurator cfg = motor.getConfigurator();
    var talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = Hardware.ARM_PIVOT_CANDI_ID;
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
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
    talonFXConfiguration.Slot0.kS = ARMPIVOT_KS;
    talonFXConfiguration.Slot0.kV = ARMPIVOT_KV;
    talonFXConfiguration.Slot0.kA = ARMPIVOT_KA;
    talonFXConfiguration.Slot0.kP = ARMPIVOT_KP;
    talonFXConfiguration.Slot0.kI = ARMPIVOT_KI;
    talonFXConfiguration.Slot0.kD = ARMPIVOT_KD;
    talonFXConfiguration.Slot0.kG = ARMPIVOT_KG;
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
