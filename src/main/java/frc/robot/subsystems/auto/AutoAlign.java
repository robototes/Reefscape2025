package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class AutoAlign {
  private static final double MaxAcceleration = 1;
  private static final double MaxAngularAcceleraition = 1;
  private static final Pose2d BRANCH_B_A =
      new Pose2d(new Translation2d(3.117, 4.181), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_B_B =
      new Pose2d(new Translation2d(3.117, 3.855), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_B_C =
      new Pose2d(new Translation2d(3.680, 2.970), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_B_D =
      new Pose2d(new Translation2d(3.975, 2.812), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_B_E =
      new Pose2d(new Translation2d(5.035, 2.706), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_B_F =
      new Pose2d(new Translation2d(5.311, 2.910), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_B_G =
      new Pose2d(new Translation2d(5.874, 3.855), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_B_H =
      new Pose2d(new Translation2d(5.874, 4.181), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_B_I =
      new Pose2d(new Translation2d(5.311, 5.092), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_B_J =
      new Pose2d(new Translation2d(5.035, 5.308), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_B_K =
      new Pose2d(new Translation2d(5.035, 5.308), Rotation2d.fromDegrees(300));
  private static final Pose2d BRANCH_B_L =
      new Pose2d(new Translation2d(3.668, 5.092), Rotation2d.fromDegrees(300));
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
      new Pose2d(new Translation2d(14.45, 3.855), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_R_B =
      new Pose2d(new Translation2d(14.45, 4.181), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_R_C =
      new Pose2d(new Translation2d(13.882, 5.200), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_R_D =
      new Pose2d(new Translation2d(13.618, 5.308), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_R_E =
      new Pose2d(new Translation2d(12.491, 5.308), Rotation2d.fromDegrees(300));
  private static final Pose2d BRANCH_R_F =
      new Pose2d(new Translation2d(12.203, 5.200), Rotation2d.fromDegrees(300));
  private static final Pose2d BRANCH_R_G =
      new Pose2d(new Translation2d(11.628, 4.181), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_R_H =
      new Pose2d(new Translation2d(11.628, 3.855), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_R_I =
      new Pose2d(new Translation2d(12.203, 2.910), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_R_J =
      new Pose2d(new Translation2d(12.491, 2.682), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_R_K =
      new Pose2d(new Translation2d(13.618, 2.682), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_R_L =
      new Pose2d(new Translation2d(13.882, 2.910), Rotation2d.fromDegrees(120));
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
    Pose2d branchPose = getClosestBranch(drivebaseSubsystem);
    boolean isBlue;
    if (!DriverStation.getAlliance().isEmpty()) {
      isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
    } else {
      isBlue = false;
    }
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
    // path.flipPath(); Returns path except it's flipped
    // this unflips it
    if (!isBlue) {
      path = path.flipPath();
    }

    return AutoBuilder.followPath(path);
  }

  public static Command autoAlign(CommandSwerveDrivetrain drivebaseSubsystem) {
    return drivebaseSubsystem.defer(() -> autoPathAlign(drivebaseSubsystem));
  }

  private static Pose2d getClosestBranch(CommandSwerveDrivetrain drivebaseSubsystem) {
    Pose2d robotPose = drivebaseSubsystem.getState().Pose;
    boolean isBlue;
    if (!DriverStation.getAlliance().isEmpty()) {
      isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
    } else {
      isBlue = false;
    }
    // figures out which branch to go to
    List<Pose2d> branchesPoses = isBlue ? blueBranchesPoses : redBranchesPoses;
    return robotPose.nearest(branchesPoses);
  }

  private static final SwerveRequest.FieldCentricFacingAngle angle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(Controls.MaxSpeed * 0.1)
          .withRotationalDeadband(Controls.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static Command aim(CommandSwerveDrivetrain drivebase, DoubleSupplier x, DoubleSupplier y) {
    return drivebase.applyRequest(
        () -> {
          // use the other method to calculate target pose
          Pose2d targetPose = getClosestBranch(drivebase);
          // calculate Rotation2d that's the angle from current Translation2d and target pose
          // Translation2d
          Pose2d currentPose = drivebase.getState().Pose;
          Translation2d distance = targetPose.getTranslation().minus(currentPose.getTranslation());
          Rotation2d targetRotation = distance.getAngle();
          // apply request
          return angle
              .withVelocityX(x.getAsDouble())
              .withVelocityY(y.getAsDouble())
              .withTargetDirection(targetRotation);
        });
  }
}
