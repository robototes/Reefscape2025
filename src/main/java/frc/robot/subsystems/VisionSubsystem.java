package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
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
  private static final double CAMERA_X_POS_METERS = Units.inchesToMeters(27.0 / 2.0 - 0.94996);
  private static final double CAMERA_Y_POS_METERS = 0;
  private static final double CAMERA_Z_POS_METERS = Units.inchesToMeters(8.12331);
  private static final double CAMERA_ROLL = 0;
  private static final double CAMERA_PITCH = Units.degreesToRadians(-30);
  private static final double CAMERA_YAW = 0;

  public static final Transform3d ROBOT_TO_CAM =
      new Transform3d(
          CAMERA_X_POS_METERS,
          CAMERA_Y_POS_METERS,
          CAMERA_Z_POS_METERS,
          new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));

  // TODO Measure these
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

  private final PhotonCamera photonCamera;
  private final PhotonCamera photonCamera2;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private final DrivebaseWrapper aprilTagsHelper;

  // These are always set with every pipeline result
  private PhotonPipelineResult latestResult = null;

  private Optional<EstimatedRobotPose> latestPose = Optional.empty();

  // These are only set when there's a valid pose
  private double lastTimestampSeconds = 0;
  private double llastTimestampSeconds = 0;
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
    photonPoseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM);

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
    shuffleboardTab.addBoolean("Has targets", this::hasTargets).withPosition(0, 0).withSize(1, 1);
    shuffleboardTab
        .addInteger("Num targets", this::getNumTargets)
        .withPosition(0, 1)
        .withSize(1, 1);
    shuffleboardTab
        .addDouble("Last timestamp", this::getLastTimestampSeconds)
        .withPosition(1, 0)
        .withSize(1, 1);
  }

  public void update() {

    for (PhotonPipelineResult result : photonCamera.getAllUnreadResults()) {
      latestResult = result;
      latestPose = photonPoseEstimator.update(latestResult);
      lastRawTimestampSeconds = latestResult.getTimestampSeconds();
      System.out.println(latestResult);

      if (latestPose.isPresent()) {
        if (lastRawTimestampSeconds > lastTimestampSeconds) {
          lastTimestampSeconds = latestPose.get().timestampSeconds;
          lastFieldPose = latestPose.get().estimatedPose.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
          aprilTagsHelper.addVisionMeasurement(lastFieldPose, lastTimestampSeconds, STANDARD_DEVS);
          robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
        }
      }

      for (PhotonPipelineResult result2 : photonCamera2.getAllUnreadResults()) {
        latestResult = result2;
        latestPose = photonPoseEstimator.update(latestResult);
        lastRawTimestampSeconds = latestResult.getTimestampSeconds();
        System.out.println(latestResult);

        if (latestPose.isPresent()) {
          if (lastRawTimestampSeconds > lastTimestampSeconds) {
            lastTimestampSeconds = latestPose.get().timestampSeconds;
            lastFieldPose = latestPose.get().estimatedPose.toPose2d();
            rawVisionFieldObject.setPose(lastFieldPose);

            aprilTagsHelper.addVisionMeasurement(
                lastFieldPose, lastTimestampSeconds, STANDARD_DEVS);
            robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
          }
        }
      }
    }
  }

  public boolean hasTargets() {
    return latestPose.isPresent();
  }

  public int getNumTargets() {
    return latestResult == null ? -1 : latestResult.getTargets().size();
  }

  /**
   * Calculates the robot pose using the best target. Returns null if there is no known robot pose.
   *
   * @return The calculated robot pose in meters.
   */
  public Pose3d getRobotPose() {
    if (latestPose.isPresent()) {
      return latestPose.get().estimatedPose;
    }
    return null;
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
