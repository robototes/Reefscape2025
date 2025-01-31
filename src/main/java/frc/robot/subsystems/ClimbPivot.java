package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {

  private final TalonFX climbPivotMotorOne;
  private final TalonFX climbPivotMotorTwo;

  public ClimbPivot() {
    climbPivotMotorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    climbPivotMotorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);

    configureMotors();
  }

  private void configureMotors() {
    var talonFXConfigurator = climbPivotMotorOne.getConfigurator();
    var talonFXConfigurator2 = climbPivotMotorTwo.getConfigurator();
    var talonFXMotorOutput = new MotorOutputConfigs();

    var currentLimits = new CurrentLimitsConfigs();

    // enable stator current limit
    currentLimits.StatorCurrentLimit = 5;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 5;
    currentLimits.SupplyCurrentLimitEnable = true;
    talonFXMotorOutput.withNeutralMode(NeutralModeValue.Brake);
    talonFXConfigurator.apply(currentLimits);
    talonFXConfigurator2.apply(currentLimits);
    talonFXConfigurator.apply(talonFXMotorOutput);
    talonFXConfigurator2.apply(talonFXMotorOutput);
  }

  public Command moveClimbMotor(double speed) {
    return run(() -> {
          climbPivotMotorOne.set(speed);
          climbPivotMotorTwo.set(-speed);
        })
        .finallyDo(
            () -> {
              climbPivotMotorOne.stopMotor();
              climbPivotMotorTwo.stopMotor();
            });
  }
}
