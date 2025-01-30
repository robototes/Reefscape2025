package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {

  private final TalonFX climbPivotMotorOne;
  private final TalonFX climbPivotMotorTwo;

  public ClimbPivot() {
    climbPivotMotorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    climbPivotMotorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);

    configureMotor();
  }

  private void configureMotor() {
    var talonFXConfigurator = climbPivotMotorOne.getConfigurator();
    var talonFXConfigurator2 = climbPivotMotorTwo.getConfigurator();

    var currentLimits = new CurrentLimitsConfigs();

    // enable stator current limit
    currentLimits.StatorCurrentLimit = 5;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 5;
    currentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigurator.apply(currentLimits);
    talonFXConfigurator2.apply(currentLimits);
  }

  public Command moveClimbMotor(double speed) {
    return runOnce(
        () -> {
          climbPivotMotorOne.set(speed);
          climbPivotMotorTwo.set(-speed);
        });
  }
}
