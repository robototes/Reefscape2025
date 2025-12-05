// THE MOST IMPORTANT PART OF THE CODE -- IMPORTS
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.armPivotConstants;
import java.util.function.Supplier;

public class ArmPivot extends SubsystemBase {

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // TalonFX
  private final TalonFX motor;

  // alerts if motor is not connected.
  private final Alert NotConnectedError =
      new Alert("ArmPivot", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  private final SysIdRoutine routine;

  private double targetPos;

  // Arm Pivot Contructor
  public ArmPivot() {
    motor = new TalonFX(armPivotConstants.ARM_PIVOT_MOTOR_ID);
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

  // ***ALL COMMANDS*** //

  // SysID quasistatic test, tests mechanism at a constant speed
  public Command SysIDQuasistatic(Direction direction) {
    return routine.quasistatic(direction);
  }

  // SysID dynamic test, tests mechanism at a linear growing speed
  public Command SysIDDynamic(Direction direction) {
    return routine.dynamic(direction);
  }

  /*  Sets the desired position to move the motor to
      - one time command using motor control
  */
  private Command setTargetPosition(double pos) {
    return runOnce(
        () -> {
          motor.setControl(m_request.withPosition(pos));
          targetPos = pos;
        });
  }

  // Boolean returning whether or not the arm is at the desired position
  public boolean atPosition(double position) {
    return MathUtil.isNear(
        position, getCurrentPosition(), armPivotConstants.ARMPIVOT_POS_TOLERANCE);
  }

  // Returns the current target position (position the arm is to move to)
  private double getTargetPosition() {
    return targetPos;
  }

  /* Returns the current position of the arm in a double measuring degrees
      - get the current positions and conversts it to a double (number with decimal)
  */
  private double getCurrentPosition() {
    if (RobotBase.isSimulation()) {
      return targetPos;
    }
    var curPos = motor.getPosition();
    return curPos.getValueAsDouble();
  }

  /*  moves arm to the input position
        - sets the target position to the inputted position
        - shrinks the difference between the current position and the target position until they are close enough to work
  */
  public Command moveToPosition(double position) {
    return setTargetPosition(position).andThen(Commands.waitUntil(atAngle(position)));
  }

  public Trigger atAngle(double position) {
    return new Trigger(
        () -> Math.abs(getCurrentPosition() - position) < armPivotConstants.ARMPIVOT_POS_TOLERANCE);
  }

  // (+) is to move arm up, and (-) is down. sets a voltage to pass to motor to move
  public Command startMovingVoltage(Supplier<Voltage> speedControl) {
    return runEnd(() -> motor.setVoltage(speedControl.get().in(Volts)), () -> motor.stopMotor());
  }

  /* logging
   * - working on logging information to shufflboard
   * - "getTab" indicates what tab it would like to edit
   * - each command adds a new trackable value with a name found in quotations marks
   */
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
    // specifies what the sensor is, what port its on, and what the gearing ratio for the sensor is
    // relative to the motor
    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = armPivotConstants.ARM_PIVOT_CANDI_ID;
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
    talonFXConfiguration.Feedback.RotorToSensorRatio = 1 / armPivotConstants.ARM_RATIO;

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 20; // previously 40
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 10; // previously 20
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // PID
    // set slot 0 gains
    talonFXConfiguration.Slot0.kS = armPivotConstants.ARMPIVOT_KS;
    talonFXConfiguration.Slot0.kV = armPivotConstants.ARMPIVOT_KV;
    talonFXConfiguration.Slot0.kA = armPivotConstants.ARMPIVOT_KA;
    talonFXConfiguration.Slot0.kP = armPivotConstants.ARMPIVOT_KP;
    talonFXConfiguration.Slot0.kI = armPivotConstants.ARMPIVOT_KI;
    talonFXConfiguration.Slot0.kD = armPivotConstants.ARMPIVOT_KD;
    talonFXConfiguration.Slot0.kG = armPivotConstants.ARMPIVOT_KG;
    talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // set Motion Magic settings in rps not mechanism units
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        320; // 160 // Target cruise velocity of 2560 rps
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration =
        320; // Target acceleration of 4960 rps/s (0.5 seconds)
    talonFXConfiguration.MotionMagic.MotionMagicJerk =
        3200; // 1600 // Target jerk of 6400 rps/s/s (0.1 seconds)

    cfg.apply(talonFXConfiguration);
  }

  // alert
  @Override
  public void periodic() {
    // Error that ensures the motor is connected
    NotConnectedError.set(notConnectedDebouncer.calculate(!motor.getMotorVoltage().hasUpdated()));
  }
}
//  -Samuel "Big North" Mallick
//  -Cleaned up and completed by Connor :)
