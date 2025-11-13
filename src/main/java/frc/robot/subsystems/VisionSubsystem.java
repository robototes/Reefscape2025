package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

/*
 * All poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station (+X is forward from blue driver station, +Y is left, +Z is up).
 */
public class VisionSubsystem extends SubsystemBase {
  // differences from center robot camera poses
  private static final double CAMERA_X_POS_METERS_LEFT = 0.26;
  private static final double CAMERA_X_POS_METERS_RIGHT = 0.27;
  private static final double CAMERA_Y_POS_METERS_LEFT = 0.25;
  private static final double CAMERA_Y_POS_METERS_RIGHT = -0.25;
  private static final double CAMERA_Z_POS_METERS_LEFT = 0.20;
  private static final double CAMERA_Z_POS_METERS_RIGHT = 0.21;
  private static final double CAMERA_ROLL_LEFT = Units.degreesToRadians(3);
  private static final double CAMERA_ROLL_RIGHT = Units.degreesToRadians(0.92);
  private static final double CAMERA_PITCH_LEFT = Units.degreesToRadians(-6.3);
  private static final double CAMERA_PITCH_RIGHT = Units.degreesToRadians(-8.3);
  private static final double CAMERA_YAW_LEFT = Units.degreesToRadians(-44.64);
  private static final double CAMERA_YAW_RIGHT = Units.degreesToRadians(46.42);
  // left camera diffrences from center robot
  public static final Transform3d ROBOT_TO_CAM_LEFT =
      new Transform3d(
          // Translation3d.kZero,
          CAMERA_X_POS_METERS_LEFT,
          CAMERA_Y_POS_METERS_LEFT,
          CAMERA_Z_POS_METERS_LEFT,
          // Rotation3d.kZero);
          new Rotation3d(CAMERA_ROLL_LEFT, CAMERA_PITCH_LEFT, CAMERA_YAW_LEFT));
  // right camera diffrences from center robot
  public static final Transform3d ROBOT_TO_CAM_RIGHT =
      new Transform3d(
          // Translation3d.kZero,
          CAMERA_X_POS_METERS_RIGHT,
          CAMERA_Y_POS_METERS_RIGHT,
          CAMERA_Z_POS_METERS_RIGHT,
          // Rotation3d.kZero);
          new Rotation3d(CAMERA_ROLL_RIGHT, CAMERA_PITCH_RIGHT, CAMERA_YAW_RIGHT));

  // Deviations
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  private static final Vector<N3> DISTANCE_SC_STANDARD_DEVS =
      VecBuilder.fill(1, 1, Units.degreesToRadians(50));

  // making the cameras, pose estimator, field2d, fieldObject2d, april tags helper objects
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;
  private final PhotonPoseEstimator photonPoseEstimatorLeftCamera;
  private final PhotonPoseEstimator photonPoseEstimatorRightCamera;
  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;
  private final DrivebaseWrapper aprilTagsHelper;

  // These are always set with every pipeline result
  private PhotonPipelineResult latestResult = null;

  // These are only set when there's a valid pose & last timestamps, last poses and distance
  // creation
  private double lastTimestampSeconds = 0;
  private double lastRawTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double Distance = 0;

  // boolean to disable vision
  // if you press the button on shuffleboard it disables vision
  private final GenericSubscriber disableVision;

  // full field pose for logging
  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  // left camera field pose for logging
  private final StructPublisher<Pose3d> rawFieldPose3dEntryLeft =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dLeft", Pose3d.struct)
          .publish();
  // right camera field pose for logging
  private final StructPublisher<Pose3d> rawFieldPose3dEntryRight =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dRight", Pose3d.struct)
          .publish();
  // field map for bot to april tag (configured for 2025)
  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // method creation for subsystem init in subsystems.java
  public VisionSubsystem(DrivebaseWrapper aprilTagsHelper) {
    // inits for field
    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    this.aprilTagsHelper = aprilTagsHelper;
    rawVisionFieldObject = robotField.getObject("RawVision");
    // cameras init hardware wise
    leftCamera = new PhotonCamera(Hardware.LEFT_CAM);
    rightCamera = new PhotonCamera(Hardware.RIGHT_CAM);
    // pose estimator inits for cameras with full field, multi-tag april tag detection and camera
    // differences from center robot
    // pose estimator is used to estimate the robot's position on the field based on the cameras
    photonPoseEstimatorLeftCamera =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM_LEFT);
    photonPoseEstimatorRightCamera =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM_RIGHT);
    // vision shuffle board tab creation
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("AprilTags");

    // last raw timestamp shuffleboard entry
    shuffleboardTab
        .addDouble("Last raw timestamp", this::getLastRawTimestampSeconds)
        .withPosition(0, 0)
        .withSize(1, 1);
    // number of april tags detected shuffleboard entry
    shuffleboardTab
        .addInteger("Num targets", this::getNumTargets)
        .withPosition(0, 1)
        .withSize(1, 1);
    // last timestamp read shuffleboard entry
    shuffleboardTab
        .addDouble("Last timestamp", this::getLastTimestampSeconds)
        .withPosition(1, 0)
        .withSize(1, 1);
    // closest tag distance in meters shuffleboard entry
    shuffleboardTab
        .addDouble("april tag distance meters", this::getDistanceToTarget)
        .withPosition(1, 1)
        .withSize(1, 1);
    // time since last reading a tag shuffleboard entry
    shuffleboardTab
        .addDouble("time since last reading", this::getTimeSinceLastReading)
        .withPosition(2, 0)
        .withSize(1, 1);
    // disable vision button if press button you disable vision in shuffleboard
    disableVision =
        shuffleboardTab
            .add("Disable vision", false)
            .withPosition(4, 0)
            .withSize(3, 2)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
  }

  // method called to update results that cameras detect
  public void update() {
    for (PhotonPipelineResult result : leftCamera.getAllUnreadResults()) {
      process(result, photonPoseEstimatorLeftCamera, rawFieldPose3dEntryLeft);
    }
    for (PhotonPipelineResult result : rightCamera.getAllUnreadResults()) {
      process(result, photonPoseEstimatorRightCamera, rawFieldPose3dEntryRight);
    }
  }

  // processs current result with cameras and camera pose estimator and where swerve believes it is
  private void process(
      PhotonPipelineResult result,
      PhotonPoseEstimator estimator,
      StructPublisher<Pose3d> rawFieldPose3dEntry) {
    // makes a different timestamp to keep track of time
    var RawTimestampSeconds = result.getTimestampSeconds();
    // for waiting until orange pi 5 syncing and getting rid old results
    if (!MathUtil.isNear(Timer.getFPGATimestamp(), RawTimestampSeconds, 5.0)) {
      return;
    }
    // updates estimated pose
    var estimatedPose = estimator.update(result);
    // if there is an actual pose then it gets a timestamp and pose3d
    if (estimatedPose.isPresent()) {
      var TimestampSeconds = estimatedPose.get().timestampSeconds;
      var FieldPose3d = estimatedPose.get().estimatedPose;
      // sets full pose3d for logging
      rawFieldPose3dEntry.set(FieldPose3d);
      // if you disabled vision then vision doesn't do anything anymore
      if (disableVision.getBoolean(false)) {
        return;
      }
      //////// below is if a tag is a specific tag id don't read this result
      // if (BadAprilTagDetector(result)) {
      //   return;
      // }
      // tolerances for rotation if its outside of tolerances then return null
      if (!MathUtil.isNear(0, FieldPose3d.getZ(), 0.10)
          || !MathUtil.isNear(0, FieldPose3d.getRotation().getX(), Units.degreesToRadians(8))
          || !MathUtil.isNear(0, FieldPose3d.getRotation().getY(), Units.degreesToRadians(8))) {
        return;
      }
      // makes a field pose for logging
      var FieldPose = FieldPose3d.toPose2d();
      int num_targets = result.targets.size();
      // gets distance
      var Distance =
          PhotonUtils.getDistanceToPose(
              FieldPose,
              // gets closed tag and gets distance
              fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d());
      // makes a pose that vision sees
      double all_of_the_distances = 0;
      for ( PhotonTrackedTarget target : result.targets) {
        var dist = PhotonUtils.getDistanceToPose(
              FieldPose,
              // gets closed tag and gets distance
              fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d());
              all_of_the_distances += dist;

      }
      all_of_the_distances = all_of_the_distances / num_targets;
      double stdDevFactor = Math.pow(all_of_the_distances, 2.0) / num_targets;
        double linearStdDev = 0.02 * stdDevFactor;
        double angularStdDev = 0.06 * stdDevFactor;
        aprilTagsHelper.addVisionMeasurement(
            FieldPose,
            RawTimestampSeconds,

            //// Use one of these, first one is current second is what advantage kit example is
            // DISTANCE_SC_STANDARD_DEVS.times(Math.max(0, distanceMeters -
            // 1)).plus(STANDARD_DEVS));
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      // sets estimated current pose to estimated vision pose
      robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
      // updates shuffleboard values
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
    // if latestResult == null then return -1 if not gets all the targets
    return latestResult == null ? -1 : latestResult.getTargets().size();
  }

  // returns lastTimestampSeconds
  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  // returns lastRawTimestampSeconds
  public double getLastRawTimestampSeconds() {
    return lastRawTimestampSeconds;
  }

  /**
   * Returns the last time we saw an AprilTag.
   *
   * @return The time we last saw an AprilTag in seconds since FPGA startup.
   */
  public double getTimeSinceLastReading() {
    return Timer.getFPGATimestamp() - lastTimestampSeconds;
  }

  // gets Distance times by 1000 which is in milimeters then gets rid of all the decimals then
  // divides to get it in meters with 4 decimal places to meters and then converts it to double
  public double getDistanceToTarget() {
    return (double) Math.round(Distance * 1000) / 1000;
  }

  // configured for 2025 reefscape to filter out any tag besides the reef tags depending on allaince
  private static boolean BadAprilTagDetector(PhotonPipelineResult r) {
    boolean isRed = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
    boolean isBlue = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
    for (var t : r.getTargets()) {
      boolean isRedReef = 6 <= t.getFiducialId() && t.getFiducialId() <= 11;
      boolean isBlueReef = 17 <= t.getFiducialId() && t.getFiducialId() <= 22;
      boolean isValid = isBlueReef && !isRed || isRedReef && !isBlue;
      if (!isValid) {
        return true;
      }
    }
    return false;
  }
}
