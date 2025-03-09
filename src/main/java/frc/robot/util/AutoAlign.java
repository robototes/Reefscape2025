package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.List;

public class AutoAlign {
  private static final double MaxAcceleration = 1;
  private static final double MaxAngularAcceleraition = 1;
  private static final Pose2d BRANCH_B_A =
      new Pose2d(new Translation2d(3.148, 4.188), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_B_B =
      new Pose2d(new Translation2d(3.148, 3.862), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_B_C =
      new Pose2d(new Translation2d(3.667, 2.941), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_B_D =
      new Pose2d(new Translation2d(3.949, 2.778), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_B_E =
      new Pose2d(new Translation2d(5.109, 2.778), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_B_F =
      new Pose2d(new Translation2d(5.315, 2.941), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_B_G =
      new Pose2d(new Translation2d(5.835, 3.862), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_B_H =
      new Pose2d(new Translation2d(5.835, 4.188), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_B_I =
      new Pose2d(new Translation2d(5.315, 5.109), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_B_J =
      new Pose2d(new Translation2d(5.109, 5.287), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_B_K =
      new Pose2d(new Translation2d(5.949, 5.287), Rotation2d.fromDegrees(300));
  private static final Pose2d BRANCH_B_L =
      new Pose2d(new Translation2d(3.667, 5.109), Rotation2d.fromDegrees(300));
  private static final List<Pose2d> blueBranchesPoses =
      Arrays.asList(
          BRANCH_B_A,
          BRANCH_B_B,
          BRANCH_B_C,
          BRANCH_B_D,
          BRANCH_B_E,
          BRANCH_B_F,
          BRANCH_B_G,
          BRANCH_B_H,
          BRANCH_B_I,
          BRANCH_B_J,
          BRANCH_B_K,
          BRANCH_B_L);

  private static final Pose2d BRANCH_R_A =
      new Pose2d(new Translation2d(14.375, 3.861), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_R_B =
      new Pose2d(new Translation2d(14.375, 4.189), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_R_C =
      new Pose2d(new Translation2d(13.865, 5.074), Rotation2d.fromDegrees(-120));
  private static final Pose2d BRANCH_R_D =
      new Pose2d(new Translation2d(13.576, 5.237), Rotation2d.fromDegrees(-120));
  private static final Pose2d BRANCH_R_E =
      new Pose2d(new Translation2d(12.537, 5.245), Rotation2d.fromDegrees(-60));
  private static final Pose2d BRANCH_R_F =
      new Pose2d(new Translation2d(12.227, 5.093), Rotation2d.fromDegrees(-60));
  private static final Pose2d BRANCH_R_G =
      new Pose2d(new Translation2d(11.738, 4.189), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_R_H =
      new Pose2d(new Translation2d(11.738, 3.842), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_R_I =
      new Pose2d(new Translation2d(12.268, 2.967), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_R_J =
      new Pose2d(new Translation2d(12.556, 2.803), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_R_K =
      new Pose2d(new Translation2d(13.576, 2.803), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_R_L =
      new Pose2d(new Translation2d(13.865, 2.967), Rotation2d.fromDegrees(120));
  private static final List<Pose2d> redBranchesPoses =
      Arrays.asList(
          BRANCH_R_A,
          BRANCH_R_B,
          BRANCH_R_C,
          BRANCH_R_D,
          BRANCH_R_E,
          BRANCH_R_F,
          BRANCH_R_G,
          BRANCH_R_H,
          BRANCH_R_I,
          BRANCH_R_J,
          BRANCH_R_K,
          BRANCH_R_L);

  private static Command autoPathAlign(CommandSwerveDrivetrain drivebaseSubsystem) {

    Pose2d robotPose = drivebaseSubsystem.getState().Pose;
    boolean isRed = AutoLogic.isRed();
    // figures out which branch to go to
    List<Pose2d> branchesPoses = isRed ? redBranchesPoses : blueBranchesPoses;
    Pose2d branchPose = robotPose.nearest(branchesPoses);
    // sets the point for the path to go to
    List<Waypoint> waypointsPoeses = PathPlannerPath.waypointsFromPoses(robotPose, branchPose);
    // creates path
    PathPlannerPath path =
        new PathPlannerPath(
            waypointsPoeses,
            new PathConstraints(
                Controls.MaxSpeed,
                MaxAcceleration,
                Controls.MaxAngularRate,
                MaxAngularAcceleraition),
            null,
            new GoalEndState(0.0, branchPose.getRotation()));
    // Theoretically:
    //   The supplier we give to AutoBuilder tells it to flip paths when we are red. (This way, the
    //   same path works for auto on either alliance.) However, in this case, we don't want to flip
    //   the path when we are red, so we flip it now to cancel it out.
    if (isRed) {
      path = path.flipPath();
    }

    return AutoBuilder.followPath(path);
  }

  public static Command autoAlign(CommandSwerveDrivetrain drivebaseSubsystem) {
    return drivebaseSubsystem.defer(() -> autoPathAlign(drivebaseSubsystem));
  }
}
