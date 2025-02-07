package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {

  private final TalonFX climbPivotMotorOne;
  private final TalonFX climbPivotMotorTwo;
  private final DigitalInput climbSensor;

  private final double CLIMB_OUT_PRESET = 90;
  private final double CLIMB_IN_PRESET = 0;
  private final double CLIMB_IN_SPEED = 0.5;
  private final double CLIMB_OUT_SPEED = -0.5;
  private boolean isClimbOut = false;
  private boolean isClimbIn = true;
  private boolean nextMoveOut = true;

  public ClimbPivot() {
    climbPivotMotorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    climbPivotMotorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);
    climbSensor = new DigitalInput(Hardware.CLIMB_SENSOR);
    configureMotors();
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
        () -> nextMoveOut);
  }

  public boolean checkClimbSensor() {
    return climbSensor.get();
  }

  @Override
  public void periodic() {
    if (climbPivotMotorOne.getPosition().getValueAsDouble() == CLIMB_OUT_PRESET) {
      isClimbOut = true;
    } else {
      isClimbOut = false;
    }
    if (climbPivotMotorOne.getPosition().getValueAsDouble() == CLIMB_IN_PRESET) {
      isClimbIn = true;
    } else {
      isClimbIn = false;
    }
  }
}
