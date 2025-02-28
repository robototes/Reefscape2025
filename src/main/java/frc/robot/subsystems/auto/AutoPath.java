package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoLogic.StartPosition;

public class AutoPath {

  private final Pose2d startPose2d;
  private StartPosition startPosition;
  private String pathPlannerAutoName;
  private final String displayName;
  private final Command autoCommand;
  private final boolean vision;
  private final PathPlannerAuto pathPlannerAuto;

  public AutoPath(String displayName, String pathPlannerAutoName, boolean vision) {
    this.displayName = displayName;
    this.pathPlannerAutoName = pathPlannerAutoName;
    pathPlannerAuto = new PathPlannerAuto(pathPlannerAutoName);
    startPose2d = pathPlannerAuto.getStartingPose();
    autoCommand = AutoBuilder.buildAuto(pathPlannerAutoName);
    this.vision = vision;

    // in the case that the auto for whatever reason's starting pose is slightly off,
    // is still able to match with a startPosition if it is close enough
    for (StartPosition pos : StartPosition.values()) {
      if (!pos.equals(StartPosition.MISC) && matchesStartPosition(pos)) {
        startPosition = pos;
        break;
      }
    }
    if (startPosition == null) {
      startPosition = StartPosition.MISC;
    }
    // debug purposes
    // System.out.println(startPosition + " " + displayName + " " + startPose2d.toString());

  }

  public AutoPath(String displayName, String pathPlannerAutoName) {
    this(displayName, pathPlannerAutoName, false);
  }

  public StartPosition getStartPose() {
    return startPosition;
  }

  public Pose2d getStartPose2d() {
    return startPose2d;
  }

  public String getAutoName() {
    return pathPlannerAutoName;
  }

  public String getDisplayName() {
    return displayName;
  }

  public Command getAutoCommand() {
    return autoCommand;
  }

  public boolean isVision() {
    return vision;
  }

  /**
   * Checks the x, y, and rotation of the auto's starting position and compares it with the expected
   * starting position, returning true if it is considered close enough to be the same.
   *
   * @param expectedStartPosition The expected start position
   * @return if it is matching
   */
  public boolean matchesStartPosition(StartPosition expectedStartPosition) {
    return (MathUtil.isNear(expectedStartPosition.startPose.getX(), startPose2d.getX(), .05)
        && MathUtil.isNear(expectedStartPosition.startPose.getY(), startPose2d.getY(), .05)
        && MathUtil.isNear(
            expectedStartPosition.startPose.getRotation().getDegrees(),
            startPose2d.getRotation().getDegrees(),
            5));
  }
}
