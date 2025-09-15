package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.libs.LimelightHelpers;
import frc.robot.libs.LimelightHelpers.LimelightResults;
import frc.robot.libs.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.libs.LimelightHelpers.RawDetection;
import frc.robot.libs.LimelightHelpers.RawFiducial;

public class LimeLight extends SubsystemBase {
  NetworkTable tableA =
          NetworkTableInstance.getDefault().getTable(Hardware.LEFT_LIMELIGHT);
  NetworkTable tableB =
          NetworkTableInstance.getDefault().getTable(Hardware.RIGHT_LIMELIGHT);
  private double lastTimestampSeconds = 0;
  private double lastRawTimestampSeconds = 0;
  private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());
  private double Distance = 0;
  public PoseEstimator m_poseEstimator;
  private final GenericSubscriber disableVision = null;
  // field map for bot to april tag (configured for 2025)
  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private Field2d robotField;

  public void update() {
    process(Hardware.LEFT_LIMELIGHT);
    process(Hardware.RIGHT_LIMELIGHT);
  }
  @SuppressWarnings("unchecked")
  private void process(String LIMELIGHT_NAME) {
    LimelightHelpers.setPipelineIndex("", 0);
    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    LimelightHelpers.setLEDMode_PipelineControl("");
    // Force LEDs on/off/blink
    LimelightHelpers.setLEDMode_ForceOn("");
    LimelightResults results = LimelightHelpers.getLatestResults("");
    
    if (disableVision.getBoolean(false)) {
      return;
    }

    
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (RawFiducial fiducial : fiducials) {
        int id = fiducial.id;                    // Tag ID
        double txnc = fiducial.txnc;             // X offset (no crosshair)
        double tync = fiducial.tync;             // Y offset (no crosshair)
        double ta = fiducial.ta;                 // Target area
        double distToCamera = fiducial.distToCamera;  // Distance to camera
        double distToRobot = fiducial.distToRobot;    // Distance to robot
        double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
    }

    // Get raw neural detector results
    RawDetection[] detections = LimelightHelpers.getRawDetections("");
    for (RawDetection detection : detections) {
        int classID = detection.classId;
        double txnc = detection.txnc;
        double tync = detection.tync;
        double ta = detection.ta;
        // Access corner coordinates if needed
        double corner0X = detection.corner0_X;
        double corner0Y = detection.corner0_Y;
        // ... corners 1-3 available similarly
        double corner1_X = detection.corner1_X;
        double corner1_Y = detection.corner1_Y;
        double corner2_X = detection.corner2_X;
        double corner2_Y = detection.corner2_Y;
        double corner3_X = detection.corner3_X; // Fourth corner X coordinate
        double corner3_Y = detection.corner3_Y; // Fourth corner Y coordinate
    }
    if (results.valid) {
    // Color/Retroreflective targets
      if (results.targets_Retro.length > 0) {
          LimelightTarget_Retro target = results.targets_Retro[0];
          Pose3d targetPose = target.getRobotPose_TargetSpace();
      }
    }
    if (mt1.rawFiducials[0].ambiguity > .7) {

    } else if (mt1.rawFiducials[0].distToCamera > 3) {

    } else if (mt1.tagCount == 0) {

    } else {

      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
      m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);

      robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }
  }
  

}
