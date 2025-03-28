package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class GroundArm extends SubsystemBase {
  public static final double STOWED_POSITION = 0.25;
  public static final double GRAB_POSITION = -0.25;
  public static final double POS_TOLERANCE = Units.degreesToRotations(5);

  // MotionMagic voltage
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // TalonFX
  private final TalonFX gndmotor;

  // TalonFX config
  public void configMotors() {
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    TalonFXConfigurator cfg = gndmotor.getConfigurator();
    var currentLimits = new CurrentLimitsConfigs();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.apply(configuration);
    // enabling stator current limits
    currentLimits.StatorCurrentLimit = 40; // subject to change
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 20; // subject to change
    currentLimits.SupplyCurrentLimitEnable = true;
    cfg.apply(currentLimits);
  }

  // position
  private double getCurrentPosition() {
    var curPos = gndmotor.getPosition();
    return curPos.getValueAsDouble();
  }

  private double targetPos;

  private double getTargetPosition() {
    return targetPos;
  }

  private Command setTargetPosition(double pos) {
    return runOnce(
        () -> {
          gndmotor.setControl(m_request.withPosition(pos));
          targetPos = pos;
        });
  }

  // logs
  private double stowedPosition;

  public void logTabs() {
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Speed", () -> gndmotor.getVelocity().getValueAsDouble());
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Motor Temperature", () -> gndmotor.getDeviceTemp().getValueAsDouble());
    Shuffleboard.getTab("Ground Arm").addDouble("Pivot Position", () -> getCurrentPosition());
    Shuffleboard.getTab("Ground Arm").addDouble("Pivot Target Pos", () -> getTargetPosition());
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Motor rotor Pos", () -> gndmotor.getRotorPosition().getValueAsDouble());
  }

  public GroundArm() {
    gndmotor = new TalonFX(Hardware.GROUND_INTAKE_ARM_MOTOR);
    configMotors();
    logTabs();
  }

  // preset command placeholder
  public Command moveToPosition(double position) {
    return setTargetPosition(position)
        .andThen(
            Commands.waitUntil(() -> Math.abs(getCurrentPosition() - position) < POS_TOLERANCE));
  }
}
