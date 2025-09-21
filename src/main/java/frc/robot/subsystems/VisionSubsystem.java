package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.libs.LimelightHelpers;
import frc.robot.libs.LimelightHelpers.PoseEstimate;


public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names
  private static final String LIMELIGHT_LEFT = Hardware.LEFT_LIMELIGHT;
  private static final String LIMELIGHT_RIGHT = Hardware.RIGHT_LIMELIGHT;

  // Enable MegaTag2 mode
  private static final boolean USE_MEGA_TAG2 = true;

  // Deviations
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  private static final Vector<N3> DISTANCE_SC_STANDARD_DEVS =
      VecBuilder.fill(1, 1, Units.degreesToRadians(50));

  // AprilTag field layout for 2025
  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private final DrivebaseWrapper aprilTagsHelper;

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private final GenericSubscriber disableVision;

  private final StructPublisher<Pose2d> fieldPose2dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose2d", Pose2d.struct)
          .publish();
  private final StructPublisher<Pose2d> rawFieldPose2dEntryLeft =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose2dLeft", Pose2d.struct)
          .publish();
  private final StructPublisher<Pose2d> rawFieldPose2dEntryRight =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose2dRight", Pose2d.struct)
          .publish();

  // state
  private double lastTimestampSeconds = 0;
  private double lastRawTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double Distance = 0;

  public VisionSubsystem(DrivebaseWrapper aprilTagsHelper) {
    this.aprilTagsHelper = aprilTagsHelper;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("AprilTags");
    shuffleboardTab.addDouble("Last raw timestamp", this::getLastRawTimestampSeconds);
    shuffleboardTab.addInteger("Num targets", this::getNumTargets);
    shuffleboardTab.addDouble("Last timestamp", this::getLastTimestampSeconds);
    shuffleboardTab.addDouble("april tag distance meters", this::getDistanceToTarget);
    shuffleboardTab.addDouble("time since last reading", this::getTimeSinceLastReading);
    shuffleboardTab.addBoolean("Using MegaTag2", () -> USE_MEGA_TAG2);

    disableVision =
        shuffleboardTab
            .add("Disable vision", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
  }

  public void update() {
    processLimelight(LIMELIGHT_LEFT, rawFieldPose2dEntryLeft);
    processLimelight(LIMELIGHT_RIGHT, rawFieldPose2dEntryRight);
  }

  private void processLimelight(String name, StructPublisher<Pose2d> rawFieldPoseEntry) {
    PoseEstimate estimate =
        USE_MEGA_TAG2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

    if (estimate == null || estimate.pose == null) {
      return;
    }

    double rawTimestampSeconds = Timer.getFPGATimestamp();
    double captureTimestampSeconds =
        rawTimestampSeconds - (estimate.latency) / 1000.0;

    Pose2d fieldPose2d = estimate.pose;
    rawFieldPoseEntry.set(fieldPose2d);

    if (disableVision.getBoolean(false)) return;

    

    // distance to closest fiducial
    double distanceMeters = Distance;
    if (estimate.tagCount > 0) {
      for (LimelightHelpers.RawFiducial rf : estimate.rawFiducials){
        double ambiguity = rf.ambiguity;
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(rf.id);
        if (tagPose.isPresent()) {
          distanceMeters =
              fieldPose2d.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
        }
        if (ambiguity == 0){
          return;
        }
      }
      
    }

    // // filter invalid tags by alliance reef
    // if (estimate.avgTagID >= 0 && isBadAprilTagForAlliance(estimate.avgTagID)) {
    //   return;
    // }

    aprilTagsHelper.addVisionMeasurement(
        fieldPose2d,
        captureTimestampSeconds,
        DISTANCE_SC_STANDARD_DEVS.times(Math.max(0, distanceMeters - 1)).plus(STANDARD_DEVS));

    if (rawTimestampSeconds > lastRawTimestampSeconds) {
      fieldPose2dEntry.set(fieldPose2d);
      lastRawTimestampSeconds = rawTimestampSeconds;
      lastTimestampSeconds = captureTimestampSeconds;
      lastFieldPose = fieldPose2d;
      rawVisionFieldObject.setPose(lastFieldPose);
      Distance = distanceMeters;
      robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
    }
  }

  public int getNumTargets() {
    int left = LimelightHelpers.getTargetCount(LIMELIGHT_LEFT);
    int right = LimelightHelpers.getTargetCount(LIMELIGHT_RIGHT);
    int total = left + right;
    return total == 0 && lastRawTimestampSeconds == 0 ? -1 : total;
  }

  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public double getLastRawTimestampSeconds() {
    return lastRawTimestampSeconds;
  }

  public double getTimeSinceLastReading() {
    return Timer.getFPGATimestamp() - lastTimestampSeconds;
  }

  public double getDistanceToTarget() {
    return (double) Math.round(Distance * 1000) / 1000;
  }

  // private static boolean isBadAprilTagForAlliance(int tagId) {
  //   // Always using Blue coordinate system, but we still block opposite reef tags
  //   boolean isRedReef = 6 <= tagId && tagId <= 11;
  //   boolean isBlueReef = 17 <= tagId && tagId <= 22;
  //   return !(isBlueReef); // only trust blue reef tags
  // }
}
