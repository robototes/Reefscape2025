package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Wrapper class around members of {@link SwerveDrive}, like the Field2d telemetry and the pose
 * estimator.
 */
public class DrivebaseWrapper {
  @FunctionalInterface
  private static interface VisionMeasurementAdder {
    void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  // Since adding a vision measurement is not thread-safe, use a functional
  // interface that can use
  // SwerveDrive's methods (which are thread-safe)
  private final VisionMeasurementAdder visionMeasurementAdder;
  private final Supplier<Pose2d> poseSupplier;

  /** Constructor for when there is no drivebase. */
  public DrivebaseWrapper() {

    // Because there's no drivebase attached, we aren't using odometry and so module
    // locations etc.
    // don't matter (but kinematics requires at least 2 module positions)
    var dummyModulePositions =
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition()};
    var poseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(new Translation2d(1, 1), new Translation2d(-1, -1)),
            new Rotation2d(),
            dummyModulePositions,
            new Pose2d());
    poseSupplier = poseEstimator::getEstimatedPosition;
    visionMeasurementAdder = poseEstimator::addVisionMeasurement;
    // The pose estimator will silently ignore updates if the internal pose buffer
    // is empty, so add
    // an odometry measurement to get started
    poseEstimator.update(Rotation2d.kZero, dummyModulePositions);
  }

  /**
   * Constructor for when there is a drivebase.
   *
   * @param swerveDrive The drivebase object to use.
   */
  public DrivebaseWrapper(CommandSwerveDrivetrain swerveDrive) {
    poseSupplier = () -> swerveDrive.getState().Pose;
    visionMeasurementAdder = swerveDrive::addVisionMeasurement;
  }

  /**
   * Adds a vision measurement with the specified standard deviations.
   *
   * @param robotPose Pose of the robot on the field in meters.
   * @param timestamp Timestamp of the vision measurement in seconds since FPGA startup.
   * @param stdDevs The standard deviations of the vision measurement. (X position in meters, Y
   *     position in meters, and heading in radians)
   * @see SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
    visionMeasurementAdder.addVisionMeasurement(robotPose, timestamp, stdDevs);
  }

  /**
   * Return the pose estimator's estimate of the robot position.
   *
   * @return The estimated pose.
   */
  public Pose2d getEstimatedPosition() {
    return poseSupplier.get();
  }
}
