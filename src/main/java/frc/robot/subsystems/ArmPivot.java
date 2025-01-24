package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ArmPivot extends SubsystemBase {

  private final TalonFX ring1;

  public ArmPivot() {
    ring1 = new TalonFX(Hardware.ARM_PIVOT_MOTOR_ID);
  }
}
