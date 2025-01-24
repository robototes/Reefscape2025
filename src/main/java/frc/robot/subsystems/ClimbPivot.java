package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbPivot extends SubsystemBase {

  private final TalonFX climbPivotMotorOne;
  private final TalonFX climbPivotMotorTwo;

  public ClimbPivot() {
    climbPivotMotorOne = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_ONE_ID);
    climbPivotMotorTwo = new TalonFX(Hardware.CLIMB_PIVOT_MOTOR_TWO_ID);
  }
}
