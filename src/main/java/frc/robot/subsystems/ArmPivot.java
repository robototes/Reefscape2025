package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import java.util.function.Supplier;

public class ArmPivot extends SubsystemBase {
  // Presets
  public static final int PRESET_L1 = 0;
  public static final int PRESET_L2_L3 = 35;
  public static final int PRESET_L4 = 90;
  public static final int HARDSTOP_HIGH = 90;
  public static final int HARDSTOP_LOW = 0;
  public static final double PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
  public static final double PLACEHOLDER_ALGAE_WEIGHT_KG = 1.5;

  // Remove once we implement PID speed
  public static int placeholderPIDSpeed;

  // TalonFX
  private final TalonFX motor;

  public ArmPivot() {
    motor = new TalonFX(Hardware.ARM_PIVOT_MOTOR_ID);
    factoryDefaults();
  }

  // (+) is to move arm up, and (-) is down
  public Command startMovingVoltage(Supplier<Voltage> speedControl) {
    return run(() -> motor.setVoltage(speedControl.get().in(Volts)));
  }

  public void moveArmCoral(int preset) {}

  public void moveArmAlgae(int preset) {}

  // TalonFX config
  public void factoryDefaults() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    TalonFXConfigurator cfg = motor.getConfigurator();
    var currentLimits = new CurrentLimitsConfigs();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.apply(configuration);
    // enabling stator current limits
    currentLimits.StatorCurrentLimit = 5; // starting low for testing
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 5; // starting low for testing
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);
  }
}
