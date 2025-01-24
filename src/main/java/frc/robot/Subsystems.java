package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import frc.robot.generated.BonkTunerConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean VISION_ENABLED = false;
  }

  // Subsystems go here

  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final VisionSubsystem visionSubsystem;

  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {
      drivebaseSubsystem = BonkTunerConstants.createDrivetrain();
    } else {
      drivebaseSubsystem = null;
    }
    if (VISION_ENABLED) {
      visionSubsystem = new VisionSubsystem();
    } else {
      visionSubsystem = null;
    }
  }
}
