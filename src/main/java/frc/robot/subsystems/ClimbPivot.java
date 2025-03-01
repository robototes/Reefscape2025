package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
  private boolean isClimbOut = false;
  private boolean isClimbIn = true;
  private boolean nextMoveOut = true;

  // alerts
  private final Alert NotConnectedErrorOne =
      new Alert("Climb", "Motor 1 not connected", AlertType.kError);
  private final Alert NotConnectedErrorTwo =
      new Alert("Climb", "Motor 2 not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncerOne = new Debouncer(.1, DebounceType.kBoth);
  private final Debouncer notConnectedDebouncerTwo = new Debouncer(.1, DebounceType.kBoth);

  public ClimbPivot() {
    motorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    motorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);
    sensor = new DigitalInput(Hardware.CLIMB_SENSOR);
    configureMotors();
    setupLogging();
    motorTwo.setControl(new Follower(motorOne.getDeviceID(), true));
  }

  private void configureMotors() {
    var talonFXConfigurator = motorOne.getConfigurator();
    var talonFXConfigurator2 = motorTwo.getConfigurator();
    TalonFXConfiguration configuration = new TalonFXConfiguration();
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
    NotConnectedErrorOne.set(
        notConnectedDebouncerOne.calculate(!motorOne.getMotorVoltage().hasUpdated()));
    NotConnectedErrorTwo.set(
        notConnectedDebouncerTwo.calculate(!motorTwo.getMotorVoltage().hasUpdated()));
  }
}
