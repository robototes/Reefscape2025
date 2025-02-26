package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

  private final TalonFX climbPivotMotorOne;
  private final TalonFX climbPivotMotorTwo;
  private final DigitalInput climbSensor;
  // entry for isClimbIn/out
  private GenericEntry climbstateEntry;
  private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climb");

  private final double CLIMB_OUT_PRESET = 90;
  private final double CLIMB_IN_PRESET = 0;
  private final double CLIMB_IN_SPEED = 0.5;
  private final double CLIMB_OUT_SPEED = -0.5;
  private final double BOOLEAN_TOLERANCE = 2;
  private boolean isClimbOut = false;
  private boolean isClimbIn = true;
  private boolean nextMoveOut = true;

  // alerts
  private final Alert NotConnectedError =
      new Alert("Climb", "Motor 1 not connected", AlertType.kError);
  private final Alert NotConnectedError2 =
      new Alert("Climb", "Motor 2 not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  public ClimbPivot() {
    climbPivotMotorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    climbPivotMotorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);
    climbSensor = new DigitalInput(Hardware.CLIMB_SENSOR);
    configureMotors();
    setupLogging();
  }

  private void configureMotors() {
    var talonFXConfigurator = climbPivotMotorOne.getConfigurator();
    var talonFXConfigurator2 = climbPivotMotorTwo.getConfigurator();
    // Set and enable current limit
    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = 5;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 5;
    currentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigurator.apply(currentLimits);
    talonFXConfigurator2.apply(currentLimits);

    var talonFXMotorOutput = new MotorOutputConfigs();
    // Enable brake mode
    talonFXMotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(talonFXMotorOutput);
    talonFXConfigurator2.apply(talonFXMotorOutput);
    // OpposeMasterDirection can be changed based on climb design, not yet sure if 2nd motor will be
    // on opposite side
    climbPivotMotorTwo.setControl(new Follower(climbPivotMotorOne.getDeviceID(), true));
  }

  public Command moveClimbMotor(double speed) {
    return run(() -> {
          climbPivotMotorOne.set(speed);
        })
        .finallyDo(
            () -> {
              climbPivotMotorOne.stopMotor();
            });
  }

  public Command toggleClimb() {
    return Commands.either(
            startEnd(
                    () -> {
                      // climb out
                      nextMoveOut = false;
                      climbPivotMotorOne.set(CLIMB_OUT_SPEED);
                    },
                    () -> {
                      climbPivotMotorOne.stopMotor();
                    })
                .until(() -> isClimbOut),
            startEnd(
                    () -> {
                      // climb in
                      nextMoveOut = true;
                      climbPivotMotorOne.set(CLIMB_IN_SPEED);
                    },
                    () -> {
                      climbPivotMotorOne.stopMotor();
                    })
                .until(() -> !isClimbIn),
            () -> nextMoveOut)
        .withName("toggleClimb");
  }

  public boolean checkClimbSensor() {
    return climbSensor.get();
  }

  public double getClimbVelocity() {
    return climbPivotMotorOne.getVelocity().getValueAsDouble();
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
  }

  @Override
  public void periodic() {
    if (MathUtil.isNear(
        climbPivotMotorOne.getPosition().getValueAsDouble(), CLIMB_OUT_PRESET, BOOLEAN_TOLERANCE)) {
      isClimbOut = true;
    } else {
      isClimbOut = false;
    }
    if (MathUtil.isNear(
        climbPivotMotorOne.getPosition().getValueAsDouble(), CLIMB_IN_PRESET, BOOLEAN_TOLERANCE)) {
      isClimbIn = true;
    } else {
      isClimbIn = false;
    }
    NotConnectedError.set(
        notConnectedDebouncer.calculate(!climbPivotMotorOne.getMotorVoltage().hasUpdated()));
    NotConnectedError2.set(
        notConnectedDebouncer.calculate(!climbPivotMotorTwo.getMotorVoltage().hasUpdated()));
  }
}
