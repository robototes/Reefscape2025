package frc.robot.subsystems.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class AutoAlign {
  public static Command autoAlignTwo(
      CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return drivebaseSubsystem
        .defer(() -> new AutoAlignTwo(drivebaseSubsystem, controls))
        .withName("Auto Align Two");
  }

  public static Boolean isBlue() {
    boolean isBlue;

    if (!DriverStation.getAlliance().isEmpty()) {
      isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
    } else {
      isBlue = false;
    }
    return isBlue;
  }

  public static boolean readyToScoreTwo() {
    return (isStationary() && isLevel() && isCloseEnoughTwo()) || oneSecondLeft();
  }

  public static boolean isStationary() {
    var speeds = AutoLogic.s.drivebaseSubsystem.getState().Speeds;
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Units.degreesToRadians(2));
  }

  public static boolean isLevel() {
    var rotation = AutoLogic.s.drivebaseSubsystem.getRotation3d();
    return MathUtil.isNear(0, rotation.getX(), Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getY(), Units.degreesToRadians(2));
  }

  public static boolean isCloseEnoughTwo() {
    var currentPose = AutoLogic.s.drivebaseSubsystem.getState().Pose;
    var branchPose = AutoAlignTwo.getNearestBranch(currentPose, isBlue());
    return currentPose.getTranslation().getDistance(branchPose.getTranslation()) < 0.05;
  }

  public static boolean
      oneSecondLeft() { // THIS WILL ONLY WORK ON THE REAL FIELD AND IN PRACTICE MODE!

    return DriverStation.getMatchTime() <= 1;
  }
}
