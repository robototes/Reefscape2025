
package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.BonkTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.TestBaseTunerConstants;
import frc.robot.subsystems.DrivebaseWrapper;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.RobotType;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean ELEVATOR_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean ARMPIVOT_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean SPINNYCLAW_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean CLIMBPIVOT_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean ELEVATOR_LED_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean GROUND_SPINNY_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean GROUND_ARM_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
  }

  // Subsystems go here
  public final DrivebaseWrapper drivebaseWrapper;
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final VisionSubsystem visionSubsystem;


  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {
      if (RobotType.getCurrent() == RobotType.BONK) {
        drivebaseSubsystem = BonkTunerConstants.createDrivetrain();
      } else if (RobotType.getCurrent() == RobotType.TESTBASE) {
        drivebaseSubsystem = TestBaseTunerConstants.createDrivetrain();
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
      SmartDashboard.putData(visionSubsystem);
    } else {
      visionSubsystem = null;
    }


    
}
}
