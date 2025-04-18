package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;

public class GroundArm extends SubsystemBase {
  private final double ARMPIVOT_KP = 40;
  private final double ARMPIVOT_KI = 0;
  private final double ARMPIVOT_KD = 0;
  private final double ARMPIVOT_KS = 0.9;
  private final double ARMPIVOT_KV = 4;
  private final double ARMPIVOT_KG = 0.048;
  private final double ARMPIVOT_KA = 0;
  public static final double STOWED_POSITION = 0.45;
  public static final double UP_POSITION =
      0.27; // untested - should be somewhere in between stowed and ground
  public static final double GROUND_POSITION = -0.050;
  public static final double QUICK_INTAKE_POSITION = 0.31;
  public static final double POS_TOLERANCE = Units.degreesToRotations(5);
  public static final double GROUND_HOLD_VOLTAGE = -0.5;
  // ratio from motor rotations to output rotations
  private static final double ARM_RATIO = 60;

  // MotionMagic voltage
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  // TalonFX
  private final TalonFX motor;

  private double targetPos;

  public GroundArm() {
    motor = new TalonFX(Hardware.GROUND_INTAKE_ARM_MOTOR);
    configMotors();
    logTabs();
  }

  // TalonFX config
  public void configMotors() {
    TalonFXConfigurator cfg = motor.getConfigurator();
    var talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = Hardware.GROUND_INTAKE_ARM_ENCODER;
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonFXConfiguration.Feedback.RotorToSensorRatio = ARM_RATIO;

    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
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

    // set Motion Magic settings in motor rps not mechanism units
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 5000;
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = 5000;
    talonFXConfiguration.MotionMagic.MotionMagicJerk = 100000;

    cfg.apply(talonFXConfiguration);
  }

  // position
  private double getCurrentPosition() {
    var curPos = motor.getPosition();
    return curPos.getValueAsDouble();
  }

  private double getTargetPosition() {
    return targetPos;
  }

  private Command setTargetPosition(double pos) {
    return runOnce(
        () -> {
          motor.setControl(m_request.withPosition(pos));
          targetPos = pos;
        });
  }

  public void logTabs() {
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Speed", () -> motor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Motor Temperature", () -> motor.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("Ground Arm").addDouble("Pivot Position", () -> getCurrentPosition());
    Shuffleboard.getTab("Ground Arm").addDouble("Pivot Target Pos", () -> getTargetPosition());
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Motor rotor Pos", () -> motor.getRotorPosition().getValueAsDouble());
  }

  // preset command placeholder
  public Command moveToPosition(double position) {
    return setTargetPosition(position).andThen(Commands.waitUntil(atPosition(position)));
  }

  public Command setVoltage(double voltage) {
    return runOnce(() -> motor.setControl(m_voltageRequest.withOutput(voltage)))
        .withName("Set voltage " + voltage);
  }

  public Trigger atPosition(double position) {
    return new Trigger(() -> Math.abs(getCurrentPosition() - position) < POS_TOLERANCE);
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor());
  }
}
