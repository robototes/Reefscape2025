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
      new Pose2d(new Translation2d(3.17, 4.22), Rotation2d.fromDegrees(-4)); // PNCMP field cal
  private static final Pose2d BRANCH_B_B =
      new Pose2d(new Translation2d(3.22, 3.82), Rotation2d.fromDegrees(0)); // WAAHS values
  private static final Pose2d BRANCH_B_C =
      new Pose2d(new Translation2d(3.69, 2.99), Rotation2d.fromDegrees(60)); // WAAHS values
  private static final Pose2d BRANCH_B_D =
      new Pose2d(new Translation2d(3.97, 2.85), Rotation2d.fromDegrees(60)); // WAAHS values
  private static final Pose2d BRANCH_B_E =
      new Pose2d(new Translation2d(5.00, 2.78), Rotation2d.fromDegrees(120)); // WAAHS values
  private static final Pose2d BRANCH_B_F =
      new Pose2d(new Translation2d(5.30, 3.01), Rotation2d.fromDegrees(120)); // WAAHS values
  private static final Pose2d BRANCH_B_G =
      new Pose2d(new Translation2d(5.80, 3.89), Rotation2d.fromDegrees(180)); // WAAHS values
  private static final Pose2d BRANCH_B_H =
      new Pose2d(new Translation2d(5.75, 4.23), Rotation2d.fromDegrees(180)); // WAAHS values
  private static final Pose2d BRANCH_B_I =
      new Pose2d(new Translation2d(5.32, 5.06), Rotation2d.fromDegrees(-125)); // PNCMP field cal
  private static final Pose2d BRANCH_B_J =
      new Pose2d(new Translation2d(4.99, 5.21), Rotation2d.fromDegrees(-119)); // PNCMP field cal
  private static final Pose2d BRANCH_B_K =
      new Pose2d(new Translation2d(3.99, 5.25), Rotation2d.fromDegrees(-63)); // PNCMP field cal
  private static final Pose2d BRANCH_B_L =
      new Pose2d(new Translation2d(3.70, 5.05), Rotation2d.fromDegrees(-59)); // PNCMP field cal
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
      new Pose2d(new Translation2d(14.41, 3.77), Rotation2d.fromDegrees(172.51)); // Practice matach log values
  private static final Pose2d BRANCH_R_B =
      new Pose2d(new Translation2d(14.31, 4.24), Rotation2d.fromDegrees(-173.25)); // WAAHS values
  private static final Pose2d BRANCH_R_C =
      new Pose2d(new Translation2d(13.88, 5.07), Rotation2d.fromDegrees(-125.86)); // Practice matach log values
  private static final Pose2d BRANCH_R_D =
      new Pose2d(new Translation2d(13.54, 5.24), Rotation2d.fromDegrees(240)); // WAAHS values
  private static final Pose2d BRANCH_R_E =
      new Pose2d(new Translation2d(12.50, 5.24), Rotation2d.fromDegrees(-56.40)); // Practice matach log values
  private static final Pose2d BRANCH_R_F =
      new Pose2d(new Translation2d(12.31, 4.97), Rotation2d.fromDegrees(-62.1)); // Practice matach log values
  private static final Pose2d BRANCH_R_G =
      new Pose2d(new Translation2d(11.73, 4.18), Rotation2d.fromDegrees(-2.35)); // WAAHS values
  private static final Pose2d BRANCH_R_H =
      new Pose2d(new Translation2d(11.88, 3.91), Rotation2d.fromDegrees(.78)); // Practice matach log values 
  private static final Pose2d BRANCH_R_I =
      new Pose2d(new Translation2d(12.29, 2.95), Rotation2d.fromDegrees(50.4)); // WAAHS values
  private static final Pose2d BRANCH_R_J =
      new Pose2d(new Translation2d(12.60, 2.82), Rotation2d.fromDegrees(49.9)); // WAAHS values
  private static final Pose2d BRANCH_R_K =
      new Pose2d(new Translation2d(13.59, 2.80), Rotation2d.fromDegrees(120)); // WAAHS values
  private static final Pose2d BRANCH_R_L =
      new Pose2d(new Translation2d(13.86, 3.03), Rotation2d.fromDegrees(121.1)); // WAAHS values
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

    return (isStationary() && isLevel() && isCloseEnough()) || oneSecondLeft();
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

  public static boolean isCloseEnough() {
    var currentPose = AutoLogic.s.drivebaseSubsystem.getState().Pose;
    var branchPose = getClosestBranch(currentPose);
    return currentPose.getTranslation().getDistance(branchPose.getTranslation()) < 0.05;
  }

  public static boolean
      oneSecondLeft() { // THIS WILL ONLY WORK ON THE REAL FIELD AND IN PRACTICE MODE!

    return DriverStation.getMatchTime() <= 1;
  }
}
