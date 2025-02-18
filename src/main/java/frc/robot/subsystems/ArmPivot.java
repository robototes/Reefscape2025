package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import java.util.function.Supplier;

public class ArmPivot extends SubsystemBase {
  // Presets
  private final double ARMPIVOT_KP = 3;
  private final double ARMPIVOT_KI = 0;
  private final double ARMPIVOT_KD = 0;
  private final double ARMPIVOT_KS = 0;
  private final double ARMPIVOT_KV = 0.12;
  private final double ARMPIVOT_KG = 0;
  private final double ARMPIVOT_KA = 0.01;
  public static final double PRESET_L1 = 0.0; // This is at position perpendicular to elevator
  public static final double PRESET_L2_L3 = 45.0;
  public static final double PRESET_L4 = 0.0;
  public static final double PRESET_UP = 90.0; // Pointing Directly upwards
  public static final double PRESET_DOWN = -90.0;
  public static final double HARDSTOP_HIGH = 95.0;
  public static final double HARDSTOP_LOW = -95.0;
  public static final double POS_TOLERANCE = 1.0;
  public static final double PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
  // Constant for gear ratio (the power that one motor gives to gear)
  private static final double ARM_RATIO = 360 * (12.0 / 60.0) * (20.0 / 60.0) * (18.0 / 48.0);
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX motor;

  private double targetPos;

  public ArmPivot() {
    motor = new TalonFX(Hardware.ARM_PIVOT_MOTOR_ID);
    factoryDefaults();
    logTabs();
  }

  // commands
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
    return run(() -> motor.setVoltage(speedControl.get().in(Volts)));
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
    var feedback_configs = new FeedbackConfigs();
    feedback_configs.FeedbackRemoteSensorID = Hardware.ARM_PIVOT_CANDI_ID;
    feedback_configs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;

    cfg.apply(feedback_configs);

    MotorOutputConfigs motorOutputConfiguration = new MotorOutputConfigs();

    motorOutputConfiguration.NeutralMode = NeutralModeValue.Brake;

    // configuration.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.apply(motorOutputConfiguration);
    // enabling current limits
    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = 20; // starting low for testing
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 10; // starting low for testing
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);

    // PID
    var slot0Configs = new Slot0Configs();
    // set slot 0 gains
    slot0Configs.kS = ARMPIVOT_KS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ARMPIVOT_KV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ARMPIVOT_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = ARMPIVOT_KP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = ARMPIVOT_KI; // no output for integrated error
    slot0Configs.kD = ARMPIVOT_KD; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.kG = ARMPIVOT_KG; // Gravity feedforward
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.apply(slot0Configs);

    // set Motion Magic settings in rps not mechanism units
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = (80); // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    cfg.apply(motionMagicConfigs);

    motor.setPosition(0);
  }
}
