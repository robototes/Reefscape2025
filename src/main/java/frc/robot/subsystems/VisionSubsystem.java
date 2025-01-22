package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;

import org.photonvision.targeting.PhotonPipelineResult;

import java.util.EnumSet;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
            Units.inchesToMeters(27.0 / 2.0 - 0.94996),
            0,
            Units.inchesToMeters(8.12331),
            new Rotation3d(0, Units.degreesToRadians(-30), 0));

    // TODO Measure these
    private static final Vector<N3> STANDARD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

    private final PhotonCamera photonCamera;
    private final PhotonCamera photonCamera2;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Field2d d2f;
    // private final DrivebaseWrapper aprilTagsHelper;
    private final FieldObject2d rawVisionFieldObject;

    // These are always set with every pipeline result
    private PhotonPipelineResult latestResult = null;
    private Optional<EstimatedRobotPose> latestPose = Optional.empty();

    // These are only set when there's a valid pose
    private double lastTimestampSeconds = 0;
    private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());

    private static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public VisionSubsystem(/* DrivebaseWrapper aprilTagsHelper */) {
        d2f = new Field2d();
        rawVisionFieldObject = d2f.getObject("RawVision");
        var networkTables = NetworkTableInstance.getDefault();
        if (Robot.isSimulation()) {
            networkTables.stopServer();
            networkTables.setServer(Hardware.PHOTON_IP);
            networkTables.startClient4("Photonvision");
        }

        photonCamera = new PhotonCamera(Hardware.FRONT_CAM);
        photonCamera2 = new PhotonCamera(Hardware.BACK_CAM);
        photonPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM);

        networkTables.addListener(
                networkTables
                        .getTable("photonvision")
                        .getSubTable(Hardware.FRONT_CAM)
                        .getEntry("rawBytes"),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> update());

        networkTables.addListener(
                networkTables
                        .getTable("photonvision")
                        .getSubTable(Hardware.BACK_CAM)
                        .getEntry("rawBytes"),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> update());

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("AprilTags");
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
        latestResult = photonCamera.getLatestResult();
        latestPose = photonPoseEstimator.update(latestResult);
        if (latestPose.isPresent()) {
            lastTimestampSeconds = latestPose.get().timestampSeconds;
            lastFieldPose = latestPose.get().estimatedPose.toPose2d();
            rawVisionFieldObject.setPose(lastFieldPose);
            // gonna fix this later
            /*
             * aprilTagsHelper.addVisionMeasurement(lastFieldPose, lastTimestampSeconds,
             * STANDARD_DEVS);
             * aprilTagsHelper.getField().setRobotPose(aprilTagsHelper.getEstimatedPosition(
             * ));
             */
        }
    }

    public boolean hasTargets() {
        return latestPose.isPresent();
    }

    public int getNumTargets() {
        return latestResult == null ? -1 : latestResult.getTargets().size();
    }

    /**
     * Calculates the robot pose using the best target. Returns null if there is no
     * known robot pose.
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
}
