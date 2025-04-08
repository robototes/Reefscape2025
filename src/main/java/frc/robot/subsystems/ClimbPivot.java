package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import java.util.function.DoubleSupplier;

public class ClimbPivot extends SubsystemBase {
  private static final boolean DUAL_MOTORS = false;

  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  public enum TargetPositions {
    STOWED,
    CLIMB_OUT,
    CLIMBED;
  }

  private final DigitalInput sensor;
  private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climb");

  private final double STOWED_MAX_PRESET = -0.447;
  private final double STOWED_MIN_PRESET = -0.450;
  private final double CLIMB_OUT_MAX_PRESET = -0.150;
  private final double CLIMB_OUT_MIN_PRESET = -0.177;
  private final double CLIMBED_MAX_PRESET = -0.325;
  private final double CLIMBED_MIN_PRESET = -0.333;
  private final double FORWARD_SOFT_STOP = -0.07;
  private final double REVERSE_SOFT_STOP = -78;
  private final double CLIMB_OUT_SPEED = 1.0;
  private final double CLIMB_HOLD_STOWED = -0.001;
  private final double CLIMB_HOLD_CLIMBOUT = -0.0;
  private final double CLIMB_HOLD_CLIMBED = -0.0705;
  private final double CLIMB_IN_SPEED = -0.75;

  // relative to eachother, likely not accurately zero'ed when obtained.x
  private static final double MIN_ROTOR_POSITION = -50.45;
  private static final double MAX_ROTOR_POSITION = 14.456;
  private static final double MIN_ENCODER_POSITION = 0.611;
  private static final double MAX_ENCODER_POSITION = 0.915;

  private boolean isClimbOut = false;
  private boolean isStowed = true;

  private TargetPositions selectedPos = TargetPositions.STOWED;
  private double maxTargetPos = STOWED_MAX_PRESET;
  private double minTargetPos = STOWED_MIN_PRESET;
  private double holdSpeed = CLIMB_HOLD_STOWED;
  private boolean moveComplete = true;
  private boolean inTolerance = true;

  private double setSpeed = 0;

  // alerts
  private final Alert NotConnectedErrorOne =
      new Alert("Climb", "Motor 1 not connected", AlertType.kError);
  private final Alert NotConnectedErrorTwo =
      new Alert("Climb", "Motor 2 not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncerOne = new Debouncer(.1, DebounceType.kBoth);
  private final Debouncer notConnectedDebouncerTwo = new Debouncer(.1, DebounceType.kBoth);

  public ClimbPivot() {
    motorLeft = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_LEFT_ID);
    if (DUAL_MOTORS) {
      motorRight = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_RIGHT_ID);
    } else {
      motorRight = null;
    }
    sensor = new DigitalInput(Hardware.CLIMB_SENSOR);
    configure();
    setupLogging();
    if (motorRight != null) {
      motorRight.setControl(new Follower(motorLeft.getDeviceID(), true));
    }
  }

  private void configure() {
    var talonFXConfigurator = motorLeft.getConfigurator();
    TalonFXConfigurator talonFXConfigurator2 = null;
    if (motorRight != null) {
      talonFXConfigurator2 = motorRight.getConfigurator();
    }

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.Feedback.FeedbackRemoteSensorID = Hardware.CLIMB_PIVOT_CANCODER_ID;
    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    configuration.Feedback.RotorToSensorRatio =
        (MAX_ROTOR_POSITION - MIN_ROTOR_POSITION) / (MAX_ENCODER_POSITION - MIN_ENCODER_POSITION);
    // Set and enable current limit
    configuration.CurrentLimits.StatorCurrentLimit = 150;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = 75;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Enable brake mode
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (motorRight != null) {
      talonFXConfigurator2.apply(configuration);
    }

    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_STOP;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_STOP;
    talonFXConfigurator.apply(configuration);
    // OpposeMasterDirection can be changed based on climb design, not yet sure if 2nd motor will be
    // on opposite side
  }

  public Command stopMotor() {
    return runOnce(() -> motorLeft.stopMotor());
  }

  private void setTargetPos(TargetPositions newTargetPos) {
    switch (newTargetPos) {
      case STOWED -> {
        selectedPos = TargetPositions.STOWED;
        maxTargetPos = STOWED_MAX_PRESET;
        minTargetPos = STOWED_MIN_PRESET;
        moveComplete = false;
      }
      case CLIMB_OUT -> {
        selectedPos = TargetPositions.CLIMB_OUT;
        maxTargetPos = CLIMB_OUT_MAX_PRESET;
        minTargetPos = CLIMB_OUT_MIN_PRESET;
        holdSpeed = CLIMB_HOLD_STOWED;
        moveComplete = false;
      }
      case CLIMBED -> {
        selectedPos = TargetPositions.CLIMBED;
        maxTargetPos = CLIMBED_MAX_PRESET;
        minTargetPos = CLIMBED_MIN_PRESET;
        holdSpeed = CLIMB_HOLD_CLIMBOUT;
        moveComplete = false;
      }
    }
  }

  public Command advanceClimbTarget() {
    return runOnce(
            () -> {
              switch (selectedPos) {
                case STOWED -> {
                  setTargetPos(TargetPositions.CLIMB_OUT);
                }
                case CLIMB_OUT -> {
                  setTargetPos(TargetPositions.CLIMBED);
                }
                case CLIMBED -> {
                  // setTargetPos(TargetPositions.STOWED);

                }
              }
            })
        .withName("Climb Sequence");
  }

  public boolean checkClimbSensor() {
    return sensor.get();
  }

  public double getClimbVelocity() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  public double getClimbPosition() {
    return motorLeft.getPosition().getValueAsDouble();
  }

  public void setPosition(double pos) {
    motorLeft.setPosition(pos);
  }

  public Command moveClimbManual(DoubleSupplier amount) {
    return runEnd(
        () -> {
          setSpeed = amount.getAsDouble();
          motorLeft.set(setSpeed);
          System.out.println(setSpeed);
        },
        () -> motorLeft.stopMotor());
  }

  public void setupLogging() {
    shuffleboardTab
        .addBoolean("Is Climb OUT?", () -> isClimbOut)
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab
        .addBoolean("Is Climb STOWED?", () -> isStowed)
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab.addString(
        "Where Move next?",
        () -> {
          switch (selectedPos) {
            case STOWED -> {
              return "Move next climbOut";
            }
            case CLIMB_OUT -> {
              return "Move next climbed";
            }
            case CLIMBED -> {
              return "Move next stowed";
            }
            default -> {
              return "-1";
            }
          }
        });
    shuffleboardTab.addDouble("targetPos", () -> maxTargetPos);
    shuffleboardTab
        .addString(
            "Where moving?",
            () -> {
              switch (selectedPos) {
                case STOWED -> {
                  return "Moving to Stow";
                }
                case CLIMB_OUT -> {
                  return "Moving to climbOut";
                }
                case CLIMBED -> {
                  return "Moving to climbed";
                }
                default -> {
                  return "-1";
                }
              }
            })
        .withWidget(BuiltInWidgets.kTextView);
    shuffleboardTab.addDouble("Set speed", () -> setSpeed);
    shuffleboardTab
        .addBoolean("Cage Detected", () -> checkClimbSensor())
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab
        .addDouble("Motor Speed", () -> getClimbVelocity())
        .withWidget(BuiltInWidgets.kTextView);
    shuffleboardTab.addDouble("Motor Position", () -> getClimbPosition());
    shuffleboardTab.addBoolean("Within Tolerance?", () -> inTolerance);
    shuffleboardTab.addBoolean("Move Complete?", () -> moveComplete);
  }

  @Override
  public void periodic() {
    double currentPos = getClimbPosition();

    if (CLIMB_OUT_MIN_PRESET <= currentPos && currentPos <= CLIMB_OUT_MAX_PRESET) {
      isClimbOut = true;
    } else {
      isClimbOut = false;
    }
    if (STOWED_MIN_PRESET <= currentPos && currentPos <= STOWED_MAX_PRESET) {
      isStowed = true;
    } else {
      isStowed = false;
    }
    if (minTargetPos <= currentPos && currentPos <= maxTargetPos) {
      inTolerance = true;
      moveComplete = true;
    } else {
      inTolerance = false;
    }
    NotConnectedErrorOne.set(
        notConnectedDebouncerOne.calculate(!motorLeft.getMotorVoltage().hasUpdated()));
    if (DUAL_MOTORS) {
      NotConnectedErrorTwo.set(
          notConnectedDebouncerTwo.calculate(!motorRight.getMotorVoltage().hasUpdated()));
    }
  }

  public Command coastMotors() {
    return startEnd(
            () -> {
              motorLeft.setNeutralMode(NeutralModeValue.Coast);
              if (DUAL_MOTORS) {
                motorRight.setNeutralMode(NeutralModeValue.Coast);
              }
            },
            () -> {
              motorLeft.setNeutralMode(NeutralModeValue.Brake);
              if (DUAL_MOTORS) {
                motorRight.setNeutralMode(NeutralModeValue.Brake);
              }
            })
        .ignoringDisable(true)
        .withName("Coast Climb");
  }

  public void brakeMotors() {
    motorLeft.setNeutralMode(NeutralModeValue.Brake);
    if (DUAL_MOTORS) {
      motorRight.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public Command advanceClimbCheck() {
    return run(
        () -> {
          if (inTolerance) {
            motorLeft.set(0);
            setSpeed = 0;
          } else {
            if (!moveComplete) {
              if (minTargetPos > getClimbPosition()) {
                motorLeft.set(CLIMB_OUT_SPEED);
                setSpeed = CLIMB_OUT_SPEED;
              } else {
                motorLeft.set(CLIMB_IN_SPEED);
                setSpeed = CLIMB_IN_SPEED;
              }
            }
          }
        });
  }

  public Trigger isClimbing() {
    return new Trigger(() -> !moveComplete);
  }

  public void moveCompleteTrue() {
    moveComplete = true;
    motorLeft.stopMotor();
  }

  public void moveCompleteFalse() {
    moveComplete = false;
  }

  public Command toStow() {
    return runOnce(() -> setTargetPos(TargetPositions.STOWED)).withName("to stow climb");
  }

  public Command toClimbOut() {
    return runOnce(() -> setTargetPos(TargetPositions.CLIMB_OUT)).withName("to climb out");
  }

  public Command toClimbed() {
    return runOnce(() -> setTargetPos(TargetPositions.CLIMBED)).withName("to climbed");
  }
}
