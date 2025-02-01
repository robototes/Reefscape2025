package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import frc.robot.generated.BonkTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.DrivebaseWrapper;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SpinnyClaw;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.RobotType;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;

    public static final boolean VISION_ENABLED = true;

    public static final boolean ELEVATOR_ENABLED = true;
    public static final boolean ARMPIVOT_ENABLED = true;
    public static final boolean SPINNYCLAW_ENABLED = true;
  }

  // Subsystems go here
  public final DrivebaseWrapper drivebaseWrapper;
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final VisionSubsystem visionSubsystem;
  public final ElevatorSubsystem elevatorSubsystem;
  public final ArmPivot armPivotSubsystem;
  public final SpinnyClaw spinnyClawSubsytem;

  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {
      if (RobotType.getCurrent() == RobotType.BONK) {
        drivebaseSubsystem = BonkTunerConstants.createDrivetrain();
      } else {
        drivebaseSubsystem = CompTunerConstants.createDrivetrain();
      }
      drivebaseWrapper = new DrivebaseWrapper(drivebaseSubsystem);
    } else {
      drivebaseSubsystem = null;
      drivebaseWrapper = new DrivebaseWrapper();
    }

    if (VISION_ENABLED) {
      visionSubsystem = new VisionSubsystem(drivebaseWrapper);
    } else {
      visionSubsystem = null;
    }
    if (ELEVATOR_ENABLED) {
      elevatorSubsystem = new ElevatorSubsystem();
    } else {
      elevatorSubsystem = null;
    }
    if (ARMPIVOT_ENABLED) {
      armPivotSubsystem = new ArmPivot();
    } else {
      armPivotSubsystem = null;
    }
    if (SPINNYCLAW_ENABLED) {
      spinnyClawSubsytem = new SpinnyClaw();
    } else {
      spinnyClawSubsytem = null;
    }
  }
}
