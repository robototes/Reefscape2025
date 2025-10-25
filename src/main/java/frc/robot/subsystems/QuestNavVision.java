package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.OptionalInt;

import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class QuestNavVision {

  public DrivebaseWrapper qWrapper = new DrivebaseWrapper();
  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private final GenericSubscriber disableVision;

  private CommandSwerveDrivetrain drivetrain = Subsystems.drivebaseSubsystem;
  private double lastTimestampSeconds = 0;
  private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);
  public double timestamp;
  public Transform2d ROBOT_TO_QUEST = new Transform2d(/* TODO: Put your x, y, z, yaw, pitch, and roll offsets here! */ );

  private QuestNav questNav = new QuestNav();
  // full field pose for logging
  private final StructPublisher<Pose2d> fieldPose3dEntry = NetworkTableInstance.getDefault()
      .getStructTopic("vision/fieldPose2d", Pose2d.struct)
      .publish();
  private Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
      0.02, // Trust down to 2cm in X direction
      0.02, // Trust down to 2cm in Y direction
      0.035 // Trust down to 2 degrees rotational
  );

  public QuestNavVision(DrivebaseWrapper qDrivebaseWrapper) {
    this.qWrapper = qDrivebaseWrapper;
    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("QuestNav");
    // disable vision button if press button you disable vision in shuffleboard
    disableVision = shuffleboardTab
        .add("Disable vision", false)
        .withPosition(4, 0)
        .withSize(3, 2)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
  }

  public void process() {
    questNav.commandPeriodic();
    OptionalInt frameCount = questNav.getFrameCount();
    if (frameCount.getAsInt() > 0) {
      // Get the latest pose data frames from the Quest
      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

      // Loop over the pose data frames and send them to the pose estimator
      for (PoseFrame questFrame : questFrames) {
        timestamp = getTimestampSeconds(questFrame);
        // Get the pose of the Quest
        Pose2d questPose = questFrame.questPose();
        Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        if (questNav.isTracking()) {
          qWrapper.addVisionMeasurement(
              robotPose, // Use the pose measurement
              questFrame.dataTimestamp(), // Use the NetworkTables timestamp
              QUESTNAV_STD_DEVS); // Your measurement uncertainty
        }
        if (timestamp > lastTimestampSeconds) {
          fieldPose3dEntry.set(questFrame.questPose());

          lastTimestampSeconds = timestamp;
          rawVisionFieldObject.setPose(lastFieldPose);
        }
      }
    }
  }

  public double getTimestampSeconds(PoseFrame questFrame) {
    double time = questFrame.dataTimestamp();
    return time;
  }
}
