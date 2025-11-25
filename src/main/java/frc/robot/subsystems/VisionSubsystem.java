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
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names
  private static final String LIMELIGHT_LEFT = Hardware.LEFT_LIMELIGHT;
  private static final String LIMELIGHT_RIGHT = Hardware.RIGHT_LIMELIGHT;
  private static final double LINEAR_STD_DEV_FACTOR = 0.02;
  private static final double STD_DEV_EXPONENT = 1.2;
  private static final double ANGULAR_STD_DEV_FACTOR = 1.6;

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
  private final LLCamera leftCamera = new LLCamera(LIMELIGHT_LEFT);
  private final LLCamera rightCamera = new LLCamera(LIMELIGHT_RIGHT);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryLeft =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dLeft", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryRight =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dRight", Pose3d.struct)
          .publish();

  // state

  private double lastTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double distance = 0;
  private double tagAmbiguity = 0;

  public VisionSubsystem(DrivebaseWrapper aprilTagsHelper) {
    this.aprilTagsHelper = aprilTagsHelper;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("AprilTags");
    shuffleboardTab
        .addDouble("Last timestamp", this::getLastTimestampSeconds)
        .withPosition(0, 0)
        .withSize(1, 1);
    shuffleboardTab
        .addInteger("Num targets", this::getNumTargets)
        .withPosition(0, 1)
        .withSize(1, 1);
    shuffleboardTab
        .addDouble("april tag distance meters", this::getDistanceToTarget)
        .withPosition(1, 1)
        .withSize(1, 1);
    shuffleboardTab
        .addDouble("time since last reading", this::getTimeSinceLastReading)
        .withPosition(2, 0)
        .withSize(1, 1);
    shuffleboardTab
        .addDouble("tag ambiguity", this::getTagAmbiguity)
        .withPosition(2, 0)
        .withSize(1, 1);

    disableVision =
        shuffleboardTab
            .add("Disable vision", false)
            .withPosition(4, 0)
            .withSize(3, 2)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
  }

  public void update() {
    processLimelight(leftCamera, rawFieldPose3dEntryLeft);
    RawFiducial[] rawFiducialsL = leftCamera.getRawFiducials();
    if (rawFiducialsL != null) {
      for (RawFiducial rf : rawFiducialsL) {
        processFiducials(rf);
      }
    }
    processLimelight(rightCamera, rawFieldPose3dEntryRight);
    RawFiducial[] rawFiducialsR = rightCamera.getRawFiducials();
    if (rawFiducialsR != null) {
      for (RawFiducial rf : rawFiducialsR) {
        processFiducials(rf);
      }
    }
  }

  private void processLimelight(
      LLCamera camera, StructPublisher<Pose3d> rawFieldPoseEntry) {
    if (disableVision.getBoolean(false)) return;
    BetterPoseEstimate estimate = camera.getBetterPoseEstimate();
  

    if (estimate != null) {
      if (estimate.tagCount <= 0) {
        return;
      }

      double timestampSeconds = estimate.timestampSeconds;
      Pose3d fieldPose3d = estimate.pose3d;
      boolean pose_bad = false;
      rawFieldPoseEntry.set(fieldPose3d);

      if (!MathUtil.isNear(0, fieldPose3d.getZ(), 0.10)
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(8))
          || !MathUtil.isNear(0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(8))) {
        pose_bad = true;
      }

      // // // filter invalid tags by alliance reef
      // if (rf.id >= 0 && BadAprilTagDetector(rf)) {
      // return;
      // }
      if (!pose_bad) {
        double stdDevFactor = Math.pow(estimate.avgTagDist, STD_DEV_EXPONENT);
        double linearStdDev = LINEAR_STD_DEV_FACTOR * stdDevFactor;
        double angularStdDev = ANGULAR_STD_DEV_FACTOR * stdDevFactor;
        aprilTagsHelper.addVisionMeasurement(
            fieldPose3d.toPose2d(),
            timestampSeconds,

            //// Use one of these, first one is current(start with STANDARD_DEVS, and for every
            // meter of distance past 1 meter,
            /// add another DISTANCE_SC_STANDARD_DEVS to the standard devs) second is what advantage
            // kit example is.
            // DISTANCE_SC_STANDARD_DEVS.times(Math.max(0, distanceMeters -
            // 1)).plus(STANDARD_DEVS));
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        robotField.setRobotPose(aprilTagsHelper.getEstimatedPosition());
      }
      if (timestampSeconds > lastTimestampSeconds) {
        if (!pose_bad) {
          fieldPose3dEntry.set(fieldPose3d);
          lastFieldPose = fieldPose3d.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
        }
        lastTimestampSeconds = timestampSeconds;
      }
    }
  }

  public int getNumTargets() {
    int L = leftCamera.getNumTargets();
    int R = rightCamera.getNumTargets();
    return L + R;
  }

  private void processFiducials(RawFiducial rf) {
    // distance to closest fiducial
    double distanceMeters = distance;
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(rf.id);
      if (tagPose.isPresent()) {
        distanceMeters = rf.distToCamera;
      }
    distance = distanceMeters;
    tagAmbiguity = rf.ambiguity;
  }


  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public double getTimeSinceLastReading() {
    return Timer.getFPGATimestamp() - lastTimestampSeconds;
  }

  public double getDistanceToTarget() {
    return (double) Math.round(distance * 1000) / 1000;
  }

  public double getTagAmbiguity() {
    return tagAmbiguity;
  }

  // private static boolean BadAprilTagDetector(LimelightHelpers.RawFiducial r) {
  //   boolean isRed = DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red));
  //   boolean isBlue =
  // DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue));
  //     boolean isRedReef = 6 <= r.id && r.id <= 11;
  //     boolean isBlueReef = 17 <= r.id && r.id <= 22;
  //     boolean isValid = isBlueReef && !isRed || isRedReef && !isBlue;
  //     if (!isValid) {
  //       return true;
  //     }

  //   return false;
  // }
}
