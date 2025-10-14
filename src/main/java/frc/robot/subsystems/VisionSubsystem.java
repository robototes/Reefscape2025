package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
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

  // Deviations
  private static final Vector<N3> STANDARD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  private static final Vector<N3> DISTANCE_SC_STANDARD_DEVS = VecBuilder.fill(1, 1, Units.degreesToRadians(50));

  // AprilTag field layout for 2025
  private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  private final DrivebaseWrapper aprilTagsHelper;

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private final GenericSubscriber disableVision;

  private final StructPublisher<Pose3d> fieldPose3dEntry = NetworkTableInstance.getDefault()
      .getStructTopic("vision/fieldPose3d", Pose3d.struct)
      .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryLeft = NetworkTableInstance.getDefault()
      .getStructTopic("vision/rawFieldPose2dLeft", Pose3d.struct)
      .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryRight = NetworkTableInstance.getDefault()
      .getStructTopic("vision/rawFieldPose3dRight", Pose3d.struct)
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
    shuffleboardTab.addDouble("Last raw timestamp", this::getLastRawTimestampSeconds).withPosition(0, 0)
        .withSize(1, 1);
    shuffleboardTab.addInteger("Num targets", this::getNumTargets).withPosition(0, 1)
        .withSize(1, 1);
    shuffleboardTab.addDouble("Last timestamp", this::getLastTimestampSeconds).withPosition(1, 0)
        .withSize(1, 1);
    shuffleboardTab.addDouble("april tag distance meters", this::getDistanceToTarget).withPosition(1, 1)
        .withSize(1, 1);
    shuffleboardTab.addDouble("time since last reading", this::getTimeSinceLastReading).withPosition(2, 0)
        .withSize(1, 1);

    disableVision = shuffleboardTab
        .add("Disable vision", false)
        .withPosition(4, 0)
        .withSize(3, 2)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
  }

  public void update() {
    processLimelight(LIMELIGHT_LEFT, rawFieldPose3dEntryLeft);
    processLimelight(LIMELIGHT_RIGHT, rawFieldPose3dEntryRight);
  }

  private void processLimelight(String name, StructPublisher<Pose3d> rawFieldPoseEntry) {
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

    if (estimate != null) {

      double rawTimestampSeconds = estimate.timestampSeconds;
      Pose3d fieldPose3d = LimelightHelpers.getBotPose3d_wpiBlue(name);
      rawFieldPoseEntry.set(fieldPose3d);
      if (disableVision.getBoolean(false))
        return;
      if (!MathUtil.isNear(0, fieldPose3d.getZ(), 0.10)
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(8))
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(8))) {
        return;
      }

      // distance to closest fiducial
      double distanceMeters = Distance;
      if (estimate.tagCount > 1) {
        for (LimelightHelpers.RawFiducial rf : estimate.rawFiducials) {
          double ambiguity = rf.ambiguity;
          Optional<Pose3d> tagPose = fieldLayout.getTagPose(rf.id);
          if (tagPose.isPresent()) {
            distanceMeters = rf.distToCamera;
          }
          if (ambiguity == 0) {
            return;
          }
        }
      }
      // // filter invalid tags by alliance reef
      // if (estimate.avgTagID >= 0 && isBadAprilTagForAlliance(estimate.avgTagID)) {
      // return;
      // }
      aprilTagsHelper.addVisionMeasurement(
          fieldPose3d.toPose2d(),
          rawTimestampSeconds,
          DISTANCE_SC_STANDARD_DEVS.times(Math.max(0, distanceMeters - 1)).plus(STANDARD_DEVS));
      robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
      if (rawTimestampSeconds > lastRawTimestampSeconds) {
        fieldPose3dEntry.set(fieldPose3d);
        lastRawTimestampSeconds = rawTimestampSeconds;
        lastFieldPose = fieldPose3d.toPose2d();
        rawVisionFieldObject.setPose(lastFieldPose);
        Distance = distanceMeters;
      }
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
  // // Always using Blue coordinate system, but we still block opposite reef tags
  // boolean isRedReef = 6 <= tagId && tagId <= 11;
  // boolean isBlueReef = 17 <= tagId && tagId <= 22;
  // return !(isBlueReef); // only trust blue reef tags
  // }
}
