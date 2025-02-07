package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import org.photonvision.targeting.PhotonPipelineResult;

/*
 * All 3D poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station (+X is forward from blue driver station, +Y is left, +Z is up).
 *
 * <p>2D field poses are different. +X is away from the driver and +Y is toward the opposing loading
 * station. Rotations are CCW+ looking down. When on the blue alliance, this means that from the
 * (blue) driver's perspective +X is away and +Y is to the left. When on the red alliance, this
 * means that from the (red) driver's perspective +X is away and +Y is to the right.
 */
public class VisionSubsystem extends SubsystemBase {
  // don't mind the value is never used part it is used
  private static final double CAMERA_X_POS_METERS_FRONT =
      Units.inchesToMeters(27.0 / 2.0 - 0.94996);
  private static final double CAMERA_X_POS_METERS_BACK =
      Units.inchesToMeters(27.0 / 2.0 - 0.94996) - 0.62;
  private static final double CAMERA_Y_POS_METERS = 0;
  private static final double CAMERA_Z_POS_METERS = Units.inchesToMeters(8.12331);
  private static final double CAMERA_ROLL = 0;
  // don't know the exact pitch for the cameras but know its around here
  private static final double CAMERA_PITCH_FRONT = Units.degreesToRadians(0);
  private static final double CAMERA_PITCH_BACK = Units.degreesToRadians(-45);
  private static final double CAMERA_YAW_FRONT = 0;
  private static final double CAMERA_YAW_BACK = Units.degreesToRadians(180);

  public static final Transform3d ROBOT_TO_CAM =
      new Transform3d(
          CAMERA_X_POS_METERS_FRONT,
          CAMERA_Y_POS_METERS,
          CAMERA_Z_POS_METERS,
          new Rotation3d(CAMERA_ROLL, CAMERA_PITCH_FRONT, CAMERA_YAW_FRONT));

  public static final Transform3d ROBOT_TO_CAM_BACK =
      new Transform3d(
          CAMERA_X_POS_METERS_BACK,
          CAMERA_Y_POS_METERS,
          CAMERA_Z_POS_METERS,
          new Rotation3d(CAMERA_ROLL, CAMERA_PITCH_BACK, CAMERA_YAW_BACK));

  // TODO Measure these
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

  private final PhotonCamera photonCamera;
  private final PhotonCamera photonCamera2;
  private final PhotonPoseEstimator photonPoseEstimatorFrontCamera;
  private final PhotonPoseEstimator photonPoseEstimatorBackCamera;
  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private final DrivebaseWrapper aprilTagsHelper;

  // These are always set with every pipeline result
  private PhotonPipelineResult latestResult = null;

  // These are only set when there's a valid pose
  private double lastTimestampSeconds = 0;
  private double lastRawTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());

  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public VisionSubsystem(DrivebaseWrapper aprilTagsHelper) {
    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    this.aprilTagsHelper = aprilTagsHelper;
    rawVisionFieldObject = robotField.getObject("RawVision");
    photonCamera = new PhotonCamera(Hardware.FRONT_CAM);
    photonCamera2 = new PhotonCamera(Hardware.BACK_CAM);
    photonPoseEstimatorFrontCamera =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM);
    photonPoseEstimatorBackCamera =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM_BACK);

    var networkTables = NetworkTableInstance.getDefault();
    networkTables.addListener(
        networkTables.getTable("photonvision").getSubTable(Hardware.FRONT_CAM).getEntry("rawBytes"),
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> update());

    networkTables.addListener(
        networkTables.getTable("photonvision").getSubTable(Hardware.BACK_CAM).getEntry("rawBytes"),
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> update());

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
  }

  private void update() {

    for (PhotonPipelineResult result : photonCamera.getAllUnreadResults()) {
      process(result, photonPoseEstimatorFrontCamera);
    }
    for (PhotonPipelineResult result : photonCamera2.getAllUnreadResults()) {
      process(result, photonPoseEstimatorBackCamera);
    }
  }

  private void process(PhotonPipelineResult result, PhotonPoseEstimator estimator) {
    var estimatedPose = estimator.update(result);
    if (estimatedPose.isPresent()) {
      var RawTimestampSeconds = result.getTimestampSeconds();
      var TimestampSeconds = estimatedPose.get().timestampSeconds;
      var FieldPose = estimatedPose.get().estimatedPose.toPose2d();
      aprilTagsHelper.addVisionMeasurement(lastFieldPose, lastTimestampSeconds, STANDARD_DEVS);
      robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
      if (RawTimestampSeconds > lastRawTimestampSeconds) {
        lastRawTimestampSeconds = RawTimestampSeconds;
        lastTimestampSeconds = TimestampSeconds;
        lastFieldPose = FieldPose;
        rawVisionFieldObject.setPose(lastFieldPose);
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
}
