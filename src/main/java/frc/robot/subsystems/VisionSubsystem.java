package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import java.util.EnumSet;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/*
 * All poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station (+X is forward from blue driver station, +Y is left, +Z is up).
 */
public class VisionSubsystem extends SubsystemBase {

  private static final double CAMERA_X_POS_METERS_FRONT = 0.25;
  private static final double CAMERA_X_POS_METERS_BACK = 0.154;
  private static final double CAMERA_Y_POS_METERS_FRONT = 0.289;
  private static final double CAMERA_Y_POS_METERS_BACK = 0.7;
  private static final double CAMERA_Z_POS_METERS_FRONT = 0.223;
  private static final double CAMERA_Z_POS_METERS_BACK = 0.737;
  private static final double CAMERA_ROLL_FRONT = Units.degreesToRadians(180);
  private static final double CAMERA_ROLL_BACK = 0;
  private static final double CAMERA_PITCH_FRONT = Units.degreesToRadians(-10);
  private static final double CAMERA_PITCH_BACK = Units.degreesToRadians(-20);
  private static final double CAMERA_YAW_FRONT = Units.degreesToRadians(-28.1);
  private static final double CAMERA_YAW_BACK = Units.degreesToRadians(180);

  // for testing only
  /*
   * private static final double CAMERA_X_POS_METERS_FRONT = 0;
   * private static final double CAMERA_X_POS_METERS_BACK = 0;
   * private static final double CAMERA_Y_POS_METERS_FRONT = 0;
   * private static final double CAMERA_Y_POS_METERS_BACK = 0;
   * private static final double CAMERA_Z_POS_METERS_FRONT = 0;
   * private static final double CAMERA_Z_POS_METERS_BACK = 0;
   * private static final double CAMERA_ROLL_FRONT = 0;
   * private static final double CAMERA_ROLL_BACK = 0;
   * private static final double CAMERA_PITCH_FRONT = 0;
   * private static final double CAMERA_PITCH_BACK = 0;
   * private static final double CAMERA_YAW_FRONT = 0;
   * private static final double CAMERA_YAW_BACK = 0;
   */

  public static final Transform3d ROBOT_TO_CAM_LEFT = Transform3d.kZero;

  public static final Transform3d ROBOT_TO_CAM_RIGHT = Transform3d.kZero;

  // TODO Measure these
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));

  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;
  private final PhotonPoseEstimator photonPoseEstimatorLeftCamera;
  private final PhotonPoseEstimator photonPoseEstimatorRightCamera;
  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private final DrivebaseWrapper aprilTagsHelper;

  // These are always set with every pipeline result
  private PhotonPipelineResult latestResult = null;

  // These are only set when there's a valid pose
  private double lastTimestampSeconds = 0;
  private double lastRawTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double Distance = 0;

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();

  private final StructPublisher<Pose3d> RawfieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawfieldPose3d", Pose3d.struct)
          .publish();

  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public VisionSubsystem(DrivebaseWrapper aprilTagsHelper) {
    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    this.aprilTagsHelper = aprilTagsHelper;
    rawVisionFieldObject = robotField.getObject("RawVision");
    leftCamera = new PhotonCamera(Hardware.LEFT_CAM);
    rightCamera = new PhotonCamera(Hardware.RIGHT_CAM);
    photonPoseEstimatorLeftCamera =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM_LEFT);
    photonPoseEstimatorRightCamera =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM_RIGHT);

    var networkTables = NetworkTableInstance.getDefault();
    networkTables.addListener(
        networkTables.getTable("photonvision").getSubTable(Hardware.LEFT_CAM).getEntry("rawBytes"),
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> update());

    // networkTables.addListener(
    //
    // networkTables.getTable("photonvision").getSubTable(Hardware.BACK_CAM).getEntry("rawBytes"),
    //     EnumSet.of(NetworkTableEvent.Kind.kValueAll),
    //     event -> update());

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("AprilTags");

    shuffleboardTab
        .addDouble("Last raw timestamp", this::getLastRawTimestampSeconds)
        .withPosition(0, 0)
        .withSize(1, 1);
    shuffleboardTab
        .addInteger("Num targets", this::getNumTargets)
        .withPosition(0, 1)
        .withSize(1, 1);
    shuffleboardTab
        .addDouble("Last timestamp", this::getLastTimestampSeconds)
        .withPosition(1, 0)
        .withSize(1, 1);
    shuffleboardTab
        .addDouble("april tag distance", this::getDistanceToTarget)
        .withPosition(1, 1)
        .withSize(1, 1);
  }

  private void update() {

    for (PhotonPipelineResult result : leftCamera.getAllUnreadResults()) {
      process(result, photonPoseEstimatorLeftCamera);
    }
    // for (PhotonPipelineResult result : backCamera.getAllUnreadResults()) {
    //   process(result, photonPoseEstimatorBackCamera);
    // }
  }

  private void process(PhotonPipelineResult result, PhotonPoseEstimator estimator) {
    var RawTimestampSeconds = result.getTimestampSeconds();
    if (!MathUtil.isNear(Timer.getFPGATimestamp(), RawTimestampSeconds, 5.0)) {
      return;
    }
    var estimatedPose = estimator.update(result);
    if (estimatedPose.isPresent()) {
      var TimestampSeconds = estimatedPose.get().timestampSeconds;
      var FieldPose3d = estimatedPose.get().estimatedPose;
      RawfieldPose3dEntry.set(FieldPose3d);
      if (!MathUtil.isNear(0, FieldPose3d.getZ(), 0.10)
          || !MathUtil.isNear(0, FieldPose3d.getRotation().getX(), Units.degreesToRadians(3))
          || MathUtil.isNear(0, FieldPose3d.getRotation().getY(), Units.degreesToRadians(3))) {
        return;
      }
      var FieldPose = FieldPose3d.toPose2d();
      var Distance =
          PhotonUtils.getDistanceToPose(
              FieldPose,
              fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d());
      aprilTagsHelper.addVisionMeasurement(FieldPose, TimestampSeconds, STANDARD_DEVS);
      robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
      if (RawTimestampSeconds > lastRawTimestampSeconds) {
        fieldPose3dEntry.set(FieldPose3d);
        lastRawTimestampSeconds = RawTimestampSeconds;
        lastTimestampSeconds = TimestampSeconds;
        lastFieldPose = FieldPose;
        rawVisionFieldObject.setPose(lastFieldPose);
        this.Distance = Distance;
      }
    }
  }

  public int getNumTargets() {
    return latestResult == null ? -1 : latestResult.getTargets().size();
  }

  /**
   * Returns the last time we saw an AprilTag.
   *
   * @return The time we last saw an AprilTag in seconds since FPGA startup.
   */
  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public double getLastRawTimestampSeconds() {
    return lastRawTimestampSeconds;
  }

  public double getDistanceToTarget() {
    return (double) Math.round(Units.metersToInches(Distance) * 10) / 10;
  }

  public static boolean allianceColor(boolean isRed) {
    if (!DriverStation.getAlliance().isEmpty()) {
      isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
      return isRed;
    } else {
      isRed = false;
      return isRed;
    }
  }
}
