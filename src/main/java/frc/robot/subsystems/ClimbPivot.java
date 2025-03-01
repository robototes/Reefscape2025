package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {

  private final TalonFX motorOne;
  private final TalonFX motorTwo;
  private CANdi encoder;

  private final DigitalInput sensor;
  // entry for isClimbIn/out
  private GenericEntry climbstateEntry;
  private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climb");

  private final double CLIMB_OUT_PRESET = -72;
  private final double FORWARD_SOFT_STOP = 1;
  private final double REVERSE_SOFT_STOP = -78;
  private final double CLIMB_IN_PRESET = 0;
  private final double CLIMB_IN_SPEED = 0.2;
  private final double CLIMB_OUT_SPEED = -0.2;
  private final double BOOLEAN_TOLERANCE = 2;
  //relative to eachother, likely not accurately zero'ed when obtained.
  private static final double MIN_ROTOR_POSITION = -50.45;
  private static final double MAX_ROTOR_POSITION = 14.456;
  private static final double MIN_ENCODER_POSITION = 0.611;
  private static final double MAX_ENCODER_POSITION = 0.915;
  private static final double GEARING_RATIO =
      (MAX_ROTOR_POSITION - MIN_ROTOR_POSITION) / (MAX_ENCODER_POSITION - MIN_ENCODER_POSITION);

  private boolean isClimbOut = false;
  private boolean isClimbIn = true;
  private boolean nextMoveOut = true;

  public ClimbPivot() {
    motorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    motorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);
    encoder = new CANdi(Hardware.CLIMB_PIVOT_CANDI_ID);
    sensor = new DigitalInput(Hardware.CLIMB_SENSOR);
    configure();
    setupLogging();
    motorTwo.setControl(new Follower(motorOne.getDeviceID(), true));
  }

  private void configure() {
    var talonFXConfigurator = motorOne.getConfigurator();
    var talonFXConfigurator2 = motorTwo.getConfigurator();

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.Feedback.FeedbackRemoteSensorID = Hardware.CLIMB_PIVOT_CANDI_ID;
    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
    configuration.Feedback.RotorToSensorRatio =
        (MAX_ROTOR_POSITION - MIN_ROTOR_POSITION) / (MAX_ENCODER_POSITION - MIN_ENCODER_POSITION);
    // Set and enable current limit
    configuration.CurrentLimits.StatorCurrentLimit = 150;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = 75;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Enable brake mode
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator2.apply(configuration);

    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_STOP;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_STOP;
    talonFXConfigurator.apply(configuration);
    // OpposeMasterDirection can be changed based on climb design, not yet sure if 2nd motor will be
    // on opposite side
  }

  public Command moveClimbMotor(double speed) {
    return run(() -> {
          motorOne.set(speed);
        })
        .finallyDo(
            () -> {
              motorOne.stopMotor();
            });
  }

  public Command toggleClimb() {
    return Commands.either(
            startEnd(
                    () -> {
                      // climb out
                      nextMoveOut = false;
                      motorOne.set(CLIMB_OUT_SPEED);
                    },
                    () -> {
                      motorOne.stopMotor();
                    })
                .until(() -> isClimbOut),
            startEnd(
                    () -> {
                      // climb in
                      nextMoveOut = true;
                      motorOne.set(CLIMB_IN_SPEED);
                    },
                    () -> {
                      motorOne.stopMotor();
                    })
                .until(() -> isClimbIn),
            () -> nextMoveOut)
        .withName("toggleClimb");
  }

  public Command holdPosition() {
    return run(() -> motorOne.set(0));
  }

  public boolean checkClimbSensor() {
    return sensor.get();
  }

  public double getClimbVelocity() {
    return motorOne.getVelocity().getValueAsDouble();
  }

  public double getClimbPosition() {
    return motorOne.getPosition().getValueAsDouble();
  }

  public void setPosition(double pos) {
    motorOne.setPosition(pos);
  }

  public Command zeroClimb() {
    return runOnce(() -> setPosition(0));
  }

  public void setupLogging() {
    shuffleboardTab
        .addBoolean("Is Climb OUT?", () -> isClimbOut)
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab
        .addBoolean("Is Climb IN?", () -> isClimbOut)
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab
        .addString(
            "Where Move next?",
            () -> {
              if (nextMoveOut) {
                return "NEXT MOVE OUT";
              } else {
                return "NEXT MOVE IN";
              }
            })
        .withWidget(BuiltInWidgets.kTextView);
    shuffleboardTab
        .addBoolean("Cage Detected", () -> checkClimbSensor())
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab
        .addDouble("Motor Speed", () -> getClimbVelocity())
        .withWidget(BuiltInWidgets.kTextView);
    shuffleboardTab.addDouble("Motor Position", () -> getClimbPosition());
  }

  @Override
  public void periodic() {
    if (MathUtil.isNear(
        motorOne.getPosition().getValueAsDouble(), CLIMB_OUT_PRESET, BOOLEAN_TOLERANCE)) {
      isClimbOut = true;
    } else {
      isClimbOut = false;
    }
    if (MathUtil.isNear(
        motorOne.getPosition().getValueAsDouble(), CLIMB_IN_PRESET, BOOLEAN_TOLERANCE)) {
      isClimbIn = true;
    } else {
      isClimbIn = false;
    }
  }
}
