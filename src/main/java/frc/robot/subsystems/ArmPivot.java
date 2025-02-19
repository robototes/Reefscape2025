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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  public static final double PRESET_L1 = 0; // This is at position perpendicular to elevator
  public static final double PRESET_L2_L3 = 0.125;
  public static final double PRESET_L4 = 0.0;
  public static final double PRESET_PRE_L4 = 1.0 / 16.0;
  public static final double PRESET_STOWED = 0.125;
  public static final double PRESET_OUT = 0;
  public static final double PRESET_UP = 0.25; // Pointing Directly upwards
  public static final double PRESET_DOWN = -0.25;
  public static final double HARDSTOP_HIGH = 0.32;
  public static final double HARDSTOP_LOW = -0.26;
  public static final double POS_TOLERANCE = 0.01;
  public static final double PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
  // Constant for gear ratio (the power that one motor gives to gear)
  private static final double ARM_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (18.0 / 48.0);
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX motor;

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

  private double getTargetPosition() {
    return targetPos;
  }

  private double getCurrentMotorPosition() {
    var curPos = motor.getPosition();
    return curPos.getValueAsDouble();
  }

  // preset command placeholder
  public Command moveToPosition(double position) {
    /*
     * if (position <= HARDSTOP_LOW) {
     * position = HARDSTOP_LOW + POS_TOLERANCE;
     * } else if (position >= HARDSTOP_HIGH) {
     * position = HARDSTOP_HIGH - POS_TOLERANCE;
     * }
     */
    // Initilizing variable to use within following Lambda
    // double
    return setTargetPosition(position)
        .andThen(
            Commands.waitUntil(
                () -> Math.abs(getCurrentMotorPosition() - position) < POS_TOLERANCE));
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
    Shuffleboard.getTab("Arm Pivot")
        .addDouble("Pivot Motor Position", () -> getCurrentMotorPosition());
    Shuffleboard.getTab("Arm Pivot").addDouble("Pivot Target Pos", () -> getTargetPosition());
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
    talonFXConfiguration.Slot0.kS = ARMPIVOT_KS; // Add 0.25 V output to overcome static friction
    talonFXConfiguration.Slot0.kV =
        ARMPIVOT_KV; // A velocity target of 1 rps results in 0.12 V output
    talonFXConfiguration.Slot0.kA =
        ARMPIVOT_KA; // An acceleration of 1 rps/s requires 0.01 V output
    talonFXConfiguration.Slot0.kP =
        ARMPIVOT_KP; // A position error of 2.5 rotations results in 12 V output
    talonFXConfiguration.Slot0.kI = ARMPIVOT_KI; // no output for integrated error
    talonFXConfiguration.Slot0.kD =
        ARMPIVOT_KD; // A velocity error of 1 rps results in 0.1 V output
    talonFXConfiguration.Slot0.kG = ARMPIVOT_KG; // Gravity feedforward
    talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // set Motion Magic settings in rps not mechanism units
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        (80); // Target cruise velocity of 80 rps
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    talonFXConfiguration.MotionMagic.MotionMagicJerk =
        1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    cfg.apply(talonFXConfiguration);
  }
}
