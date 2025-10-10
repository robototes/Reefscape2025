package frc.robot.subsystems.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.ScoringMode;
import java.util.List;

/**
 * Command that automatically moves the elevator to the proper height for the nearest algae based on
 * alliance and current robot pose.
 */
public class AutoAlgaeHeights extends Command {

  private final CommandSwerveDrivetrain drivebase;
  private final SuperStructure s;
  private final Controls c;

  private static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final List<Pose2d> blueAlgaePoses =
      List.of(
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(22).get().toPose2d());

  private static final List<Pose2d> redAlgaePoses =
      List.of(
          aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(11).get().toPose2d());

  public AutoAlgaeHeights(CommandSwerveDrivetrain drivebase, SuperStructure s, Controls c) {
    this.drivebase = drivebase;
    this.s = s;
    this.c = c;
  }

  public static Command autoAlgaeIntakeCommand(
      CommandSwerveDrivetrain drivebase, SuperStructure s, Controls c) {
    return new AutoAlgaeHeights(drivebase, s, c);
  }

  private static boolean isBlue() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance.equals(Alliance.Blue))
        .orElse(false);
  }

  private static Pose2d getNearestAlgae(Pose2d robotPose) {
    List<Pose2d> poses = isBlue() ? blueAlgaePoses : redAlgaePoses;
    return robotPose.nearest(poses);
  }

  private String aprilTagToAlgaeHeight(Pose2d algaePose) {
    // Map poses to two heights only
    List<Pose2d> poses = isBlue() ? blueAlgaePoses : redAlgaePoses;
    int index = poses.indexOf(algaePose);

    if (index >= 3) {
      return "top"; // upper reef
    } else {
      return "low"; // lower reef
    }
  }

  private String activeCommand = "";

  @Override
  public void execute() {
    Pose2d robotPose = drivebase.getState().Pose;
    Pose2d nearestAlgae = getNearestAlgae(robotPose);

    String targetHeight = aprilTagToAlgaeHeight(nearestAlgae);

    if (!(activeCommand.equals(targetHeight))) {
      if (targetHeight.equals("top")) {
        s.algaeLevelThreeFourIntake().schedule();
        activeCommand = "top";
      } else {
        s.algaeLevelTwoThreeIntake().schedule();
        activeCommand = "low";
      }
    }
  }

  @Override
  public boolean isFinished() {
    return c.intakeMode != ScoringMode.ALGAE;
  }
}
