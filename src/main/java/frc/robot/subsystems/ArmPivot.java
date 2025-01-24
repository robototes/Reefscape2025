package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ArmPivot extends SubsystemBase {
  // Presets
  public static int PRESET_L1 = 0;
  public static int PRESET_L2_L3 = 35;
  public static int PRESET_L4;
  public static int HARDSTOP_HIGH = 90;
  public static int HARDSTOP_LOW = 0;
  public static double placeholderCoralWeight;
  public static double placeholderAlgaeWeight;

  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX ring1;
  private TalonFXConfiguration configuration = new TalonFXConfiguration();

  public ArmPivot() {
    ring1 = new TalonFX(Hardware.ARM_PIVOT_MOTOR_ID);
    factoryDefaults();
  }

  public void moveArmCoral(int preset) {}

  public void moveArmAlgae(int preset) {}

  // TalonFX config
  public void factoryDefaults() {
    TalonFXConfigurator cfg = motor.getConfigurator();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.apply(configuration);
  }
}
