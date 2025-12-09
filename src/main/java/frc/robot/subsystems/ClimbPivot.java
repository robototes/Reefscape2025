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
import frc.robot.Constants.ClimbConstants;
import java.util.function.DoubleSupplier;

public class ClimbPivot extends SubsystemBase {
  // variable that allows you to switch between using two motors or just one
  private static final boolean DUAL_MOTORS = false;

  // the two motor instance variables
  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  // the three target positions in an enum
  public enum TargetPositions {
    STOWED,
    CLIMB_OUT,
    CLIMBED;
  }

  // digitalInput for the sensor (never worked lol)
  public final DigitalInput sensor;

  // variable for the shuffleboard tab for consistency
  private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climb");

  // two status variables
  public boolean isClimbOut = false;
  private boolean isStowed = true;

  // setting the starting target position
  private TargetPositions selectedPos = TargetPositions.STOWED;
  private double maxTargetPos = ClimbConstants.STOWED_MAX_PRESET;
  private double minTargetPos = ClimbConstants.STOWED_MIN_PRESET;
  private double holdSpeed = ClimbConstants.CLIMB_HOLD_STOWED;

  // if moveComplete is true it wont move regardless of if its in range. This is to ensure that
  // when disabled, when re-enabled it doesnt start moving.
  private boolean moveComplete = true;

  // inTolerenace is used to make sure that we're within a ok range between our min goal and max
  // goal
  private boolean inTolerance = true;

  // the target speed
  private double setSpeed = 0;
  private double speedToSet = 0;
  private double pError = 0;

  // alerts for checking if either motor is or isnt connected
  private final Alert NotConnectedErrorOne =
      new Alert("Climb", "Motor 1 not connected", AlertType.kError);
  private final Alert NotConnectedErrorTwo =
      new Alert("Climb", "Motor 2 not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncerOne = new Debouncer(.1, DebounceType.kBoth);
  private final Debouncer notConnectedDebouncerTwo = new Debouncer(.1, DebounceType.kBoth);

  // here we go
  public ClimbPivot() {
    motorLeft = new TalonFX(ClimbConstants.CLIMB_PIVOT_MOTOR_LEFT_ID);
    // checking to instantiate the 2nd motor if two are installed. If only one is installed, the 2nd
    // one is set to null.
    if (DUAL_MOTORS) {
      motorRight = new TalonFX(ClimbConstants.CLIMB_PIVOT_MOTOR_RIGHT_ID);
    } else {
      motorRight = null;
    }

    // the failed climb sensor
    sensor = new DigitalInput(ClimbConstants.CLIMB_SENSOR);

    // motor configuration and shuffleboard logging methods
    configure();
    setupLogging();

    // if motor 2 exists, setting its contorl mode as a follower to the 1st motor through CTRE
    if (motorRight != null) {
      motorRight.setControl(new Follower(motorLeft.getDeviceID(), true));
    }
  }

  // method for configuring the two motors
  private void configure() {

    // ctre uses configurator classes to configure the motors
    var talonFXConfigurator = motorLeft.getConfigurator();

    // making the 2nd configurator null, unless the 2nd motor actually exist
    TalonFXConfigurator talonFXConfigurator2 = null;
    if (motorRight != null) {
      talonFXConfigurator2 = motorRight.getConfigurator();
    }
    // one configuration to apply to both motors with the values that apply to both of them
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    // remote sensor values for the WCP encoder
    configuration.Feedback.FeedbackRemoteSensorID = ClimbConstants.CLIMB_PIVOT_CANCODER_ID;
    configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // previously mentioned ratio calculation
    configuration.Feedback.RotorToSensorRatio =
        (ClimbConstants.MAX_ROTOR_POSITION - ClimbConstants.MIN_ROTOR_POSITION)
            / (ClimbConstants.MAX_ENCODER_POSITION - ClimbConstants.MIN_ENCODER_POSITION);
    // Set and enable current limit
    configuration.CurrentLimits.StatorCurrentLimit = ClimbConstants.CLIMB_STATOR_CURRENT_LIMIT;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = ClimbConstants.CLIMB_SUPPLY_CURRENT_LIMIT;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // Enable brake mode
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // applying the configuration if the 2nd motor exists
    // this was applied here before the rest of the config so the following configs arent applied
    if (motorRight != null) {
      talonFXConfigurator2.apply(configuration);
    }

    // software limits employed after the 2nd motor configs were applied so it doesnt get applied to
    // the 2nd motor
    // having softlimits enabled on both motors isnt great
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.FORWARD_SOFT_STOP;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.REVERSE_SOFT_STOP;
    talonFXConfigurator.apply(configuration);
    // OpposeMasterDirection can be changed based on climb design, not yet sure if 2nd motor will be
    // on opposite side
  }

  // command to stop the motor
  public Command stopMotor() {
    return runOnce(() -> motorLeft.stopMotor());
  }

  // method to set the target position based on what target position the enum parameter is
  private void setTargetPos(TargetPositions newTargetPos) {
    switch (newTargetPos) {
      case STOWED -> {
        // if stowed, it sets the pos its going to as stowed, and sets the target positions
        // also makes move complete false to indicate it will move
        selectedPos = TargetPositions.STOWED;
        maxTargetPos = ClimbConstants.STOWED_MAX_PRESET;
        minTargetPos = ClimbConstants.STOWED_MIN_PRESET;
        moveComplete = false;
      }
      case CLIMB_OUT -> {
        // if climb_out, it sets the pos its going to as climb_out, and sets the target positions
        // also makes move complete false to indicate it will move
        selectedPos = TargetPositions.CLIMB_OUT;
        maxTargetPos = ClimbConstants.CLIMB_OUT_MAX_PRESET;
        minTargetPos = ClimbConstants.CLIMB_OUT_MIN_PRESET;
        holdSpeed = ClimbConstants.CLIMB_HOLD_STOWED;
        moveComplete = false;
      }
      case CLIMBED -> {
        // if climbed, it sets the pos its going to as climbed, and sets the target positions
        // also makes move complete false to indicate it will move
        selectedPos = TargetPositions.CLIMBED;
        maxTargetPos = ClimbConstants.CLIMBED_MAX_PRESET;
        minTargetPos = ClimbConstants.CLIMBED_MIN_PRESET;
        holdSpeed = ClimbConstants.CLIMB_HOLD_CLIMBOUT;
        moveComplete = false;
      }
    }
  }

  // method for setting the climb target depending on what the selectedPosition is
  public Command advanceClimbTarget() {
    return runOnce(
            () -> {
              // depending on the switch itll run setTargetPos to a different position except stowed
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

  // check the digital input climb sensor
  public boolean checkClimbSensor() {
    return !sensor.get();
  }

  // get the velocity from the motor
  public double getClimbVelocity() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  // get the position from the encoder (I think?)
  public double getClimbPosition() {
    return motorLeft.getPosition().getValueAsDouble();
  }

  // set what position its at (if you need to zero it)
  public void setPosition(double pos) {
    motorLeft.setPosition(pos);
  }

  // manual climb movement
  public Command moveClimbManual(DoubleSupplier amount) {
    // runs the command, and then run the next method when it ends
    return runEnd(
        () -> {
          // set speed to the input amount
          setSpeed = amount.getAsDouble();
          // applies speed
          motorLeft.set(setSpeed);
          // spam console (should've removed this...)
          System.out.println(setSpeed);
        },
        // when the command ends, stop the motors
        () -> motorLeft.stopMotor());
  }

  // basic logging
  public void setupLogging() {
    shuffleboardTab
        .addBoolean("Is Climb OUT?", () -> isClimbOut)
        .withWidget(BuiltInWidgets.kBooleanBox);
    shuffleboardTab
        .addBoolean("Is Climb STOWED?", () -> isStowed)
        .withWidget(BuiltInWidgets.kBooleanBox);

    // structured similarly to setTargetPos, mimics that to log what state its in
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

    // structured similarly to setTargetPos, mimics that to log what state its in
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
    // gets the current position at the time of that periodic rerun
    double currentPos = getClimbPosition();

    // checks what state its in position wise and returns if its in that state
    if (ClimbConstants.CLIMB_OUT_MIN_PRESET <= currentPos
        && currentPos <= ClimbConstants.CLIMB_OUT_MAX_PRESET) {
      isClimbOut = true;
    } else {
      isClimbOut = false;
    }
    if (ClimbConstants.STOWED_MIN_PRESET <= currentPos
        && currentPos <= ClimbConstants.STOWED_MAX_PRESET) {
      isStowed = true;
    } else {
      isStowed = false;
    }
    // checks if its in tolerance. if its in tolerance it ends the movement. otherwise, keep going
    if (minTargetPos <= currentPos && currentPos <= maxTargetPos) {
      inTolerance = true;
      moveComplete = true;
    } else {
      inTolerance = false;
    }

    // errors checking if the motor lost connection
    NotConnectedErrorOne.set(
        notConnectedDebouncerOne.calculate(!motorLeft.getMotorVoltage().hasUpdated()));
    if (DUAL_MOTORS) {
      NotConnectedErrorTwo.set(
          notConnectedDebouncerTwo.calculate(!motorRight.getMotorVoltage().hasUpdated()));
    }
  }

  // coasts the motors by changing their neutralmode
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

  // brakes the motors the same way as the previous method
  public void brakeMotors() {
    motorLeft.setNeutralMode(NeutralModeValue.Brake);
    if (DUAL_MOTORS) {
      motorRight.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  // checks if its in tolerance, then stops the movement. otherwise, if the target position is ahead
  // of it move forward, if its behind it, move back
  public Command advanceClimbCheck() {
    return run(
        () -> {
          if (inTolerance) {
            motorLeft.set(0);
            setSpeed = 0;
          } else {
            if (!moveComplete) {
              pError = minTargetPos - getClimbPosition();
              if (pError > 0) {
                speedToSet = ClimbConstants.CLIMB_OUT_SPEED * pError * ClimbConstants.climbOutKp;
                motorLeft.set(speedToSet);
                setSpeed = speedToSet;
              } else {
                speedToSet = ClimbConstants.CLIMB_IN_SPEED * -pError * ClimbConstants.climbInKp;
                motorLeft.set(speedToSet);
                setSpeed = speedToSet;
              }
            }
          }
        });
  }

  // a trigger for checking if we're climbing
  public Trigger isClimbing() {
    return new Trigger(() -> !moveComplete);
  }

  // a value that just sets moveComplete to true and stops the motors
  public void moveCompleteTrue() {
    moveComplete = true;
    motorLeft.stopMotor();
  }

  // sets move complete to false
  public void moveCompleteFalse() {
    moveComplete = false;
  }

  // sets the target position to go to stowed positions
  public Command toStow() {
    return runOnce(() -> setTargetPos(TargetPositions.STOWED)).withName("to stow climb");
  }

  // sets the target position to go to climb_out position
  public Command toClimbOut() {
    return runOnce(() -> setTargetPos(TargetPositions.CLIMB_OUT)).withName("to climb out");
  }

  // sets the target position to go to climbed position
  public Command toClimbed() {
    return runOnce(() -> setTargetPos(TargetPositions.CLIMBED)).withName("to climbed");
  }

  public Trigger cageDetected() {
    return new Trigger(() -> checkClimbSensor())
        .debounce(0.1, DebounceType.kRising)
        .debounce(0.05, DebounceType.kFalling);
  }
}
