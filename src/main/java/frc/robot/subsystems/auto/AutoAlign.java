package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class AutoAlign {
  private static final double MaxAcceleration = 1;
  private static final double MaxAngularAcceleraition = 1;
  private static final Pose2d BRANCH_B_A =
      new Pose2d(new Translation2d(3.200, 4.190), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_B_B =
      new Pose2d(new Translation2d(3.200, 3.860), Rotation2d.fromDegrees(0));
  private static final Pose2d BRANCH_B_C =
      new Pose2d(new Translation2d(3.700, 2.990), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_B_D =
      new Pose2d(new Translation2d(3.980, 2.820), Rotation2d.fromDegrees(60));
  private static final Pose2d BRANCH_B_E =
      new Pose2d(new Translation2d(4.990, 2.820), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_B_F =
      new Pose2d(new Translation2d(5.270, 2.990), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_B_G =
      new Pose2d(new Translation2d(5.775, 3.860), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_B_H =
      new Pose2d(new Translation2d(5.775, 4.190), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_B_I =
      new Pose2d(new Translation2d(5.270, 5.060), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_B_J =
      new Pose2d(new Translation2d(4.990, 5.230), Rotation2d.fromDegrees(240));
  private static final Pose2d BRANCH_B_K =
      new Pose2d(new Translation2d(3.980, 5.230), Rotation2d.fromDegrees(300));
  private static final Pose2d BRANCH_B_L =
      new Pose2d(new Translation2d(3.700, 5.060), Rotation2d.fromDegrees(300));
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
      new Pose2d(new Translation2d(14.490, 3.930), Rotation2d.fromDegrees(179.3));
  private static final Pose2d BRANCH_R_B =
      new Pose2d(new Translation2d(14.350, 4.190), Rotation2d.fromDegrees(180));
  private static final Pose2d BRANCH_R_C =
      new Pose2d(new Translation2d(13.85, 5.04), Rotation2d.fromDegrees(-124));
  private static final Pose2d BRANCH_R_D =
      new Pose2d(new Translation2d(13.59, 5.2), Rotation2d.fromDegrees(-119));
  private static final Pose2d BRANCH_R_E =
      new Pose2d(new Translation2d(12.510, 5.250), Rotation2d.fromDegrees(-54));
  private static final Pose2d BRANCH_R_F =
      new Pose2d(new Translation2d(12.56, 5.27), Rotation2d.fromDegrees(300));
  private static final Pose2d BRANCH_R_G =
      new Pose2d(new Translation2d(11.740, 4.240), Rotation2d.fromDegrees(-2.35));
  private static final Pose2d BRANCH_R_H =
      new Pose2d(new Translation2d(11.780, 3.920), Rotation2d.fromDegrees(-4.9));
  private static final Pose2d BRANCH_R_I =
      new Pose2d(new Translation2d(12.205, 2.980), Rotation2d.fromDegrees(50.4));
  private static final Pose2d BRANCH_R_J =
      new Pose2d(new Translation2d(12.440, 2.850), Rotation2d.fromDegrees(49.9));
  private static final Pose2d BRANCH_R_K =
      new Pose2d(new Translation2d(13.570, 2.820), Rotation2d.fromDegrees(120));
  private static final Pose2d BRANCH_R_L =
      new Pose2d(new Translation2d(13.920, 2.950), Rotation2d.fromDegrees(121.1));
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
    Pose2d branchPose = getClosestBranch(robotPose);
    Transform2d robotToBranch = branchPose.minus(robotPose);
    if (robotToBranch.getTranslation().getNorm() < 0.01
        && Math.abs(robotToBranch.getRotation().getDegrees()) < 1) {
      return Commands.none();
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
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public static Command autoAlign(CommandSwerveDrivetrain drivebaseSubsystem) {
    return drivebaseSubsystem.defer(() -> autoPathAlign(drivebaseSubsystem)).withName("Auto Align");
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

  private static Pose2d getClosestBranch(Pose2d robotPose) {

    // figures out which branch to go to
    List<Pose2d> branchesPoses = isBlue() ? blueBranchesPoses : redBranchesPoses;
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
          Pose2d currentPose = drivebase.getState().Pose;
          Pose2d targetPose = getClosestBranch(currentPose);
          // calculate Rotation2d that's the angle from current Translation2d and target pose
          // Translation2d
          Translation2d distance = targetPose.getTranslation().minus(currentPose.getTranslation());
          Rotation2d targetRotation = distance.getAngle();
          // apply request
          return angle
              .withVelocityX(x.getAsDouble())
              .withVelocityY(y.getAsDouble())
              .withTargetDirection(targetRotation);
        });
  }

  public static boolean readyToScore() {
    var currentPose = AutoLogic.s.drivebaseSubsystem.getState().Pose;
    var branchPose = getClosestBranch(currentPose);
    var speeds = AutoLogic.s.drivebaseSubsystem.getState().Speeds;
    var rotation = AutoLogic.s.drivebaseSubsystem.getRotation3d();
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getX(), Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getY(), Units.degreesToRadians(2))
        && currentPose.getTranslation().getDistance(branchPose.getTranslation()) < 0.05;
  }
}
