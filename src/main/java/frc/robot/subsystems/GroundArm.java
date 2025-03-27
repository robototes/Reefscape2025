package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class GroundArm extends SubsystemBase {
  public static final double STOWED_POSITION = 0.25;
  public static final double GRAB_POSITION = -0.25;

//MotionMagic voltage
private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

// TalonFX
  private final TalonFX gndmotor;

//target position
  private double targetPos;
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
    Shuffleboard.getTab("Ground Arm").addDouble("Pivot Target Pos", () -> targetPos());
    Shuffleboard.getTab("Ground Arm")
        .addDouble("Pivot Motor rotor Pos", () -> gndmotor.getRotorPosition().getValueAsDouble());
  }

     public GroundArm() {
    gndmotor = new TalonFX(Hardware.SPINNY_CLAW_MOTOR_ID);
    configMotors();
    logTabs();
  }

  private Command armPos(double pos) {
    return startEnd(
        () -> {
          gndmotor.setVoltage(pos);
           stowedPosition = pos;
        },
        () -> gndmotor.stopMotor());
   }

   public Command stowedPosition() {
    return armPos(STOWED_POSITION).withName("holdIntakePower");
   }
}