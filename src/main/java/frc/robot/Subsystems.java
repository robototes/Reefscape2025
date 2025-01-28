package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import frc.robot.generated.BonkTunerConstants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ClimbPivot;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean ARMPIVOT_ENABLED = true;
    public static final boolean CLIMBPIVOT_ENABLED = true;
  }

  // Subsystems go here

  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final ArmPivot armPivotSubsystem;
  public final ClimbPivot climbPivotSubsystem;
  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {
      drivebaseSubsystem = BonkTunerConstants.createDrivetrain();
    } else {
      drivebaseSubsystem = null;
    }

    if (ARMPIVOT_ENABLED) {
      armPivotSubsystem = new ArmPivot();
    } else {
      armPivotSubsystem = null;
    }

    if (CLIMBPIVOT_ENABLED){
      climbPivotSubsystem = new ClimbPivot();
    } else {
      climbPivotSubsystem = null;
    }
  }
}
