package frc.robot.libs;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.libs.LimelightHelpers.PoseEstimate;
import frc.robot.libs.LimelightHelpers.RawDetection;
import frc.robot.libs.LimelightHelpers.RawFiducial;

public class LLCamera {
  private static String name;

  public LLCamera(String name) {
    this.name = name;
  }

  public LLCamera() {
    this("limelight");
  }

  public int getNumTargets() {
    int tar = LimelightHelpers.getTargetCount(name);
    return tar;
  }

  public boolean hasValidTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getHoritontalOffset() {
    return LimelightHelpers.getTX(name);
  }

  public double getVerticalOffset(String limelightName) {
    return LimelightHelpers.getTY(name);
  }

  public double getTargetArea() {
    return LimelightHelpers.getTA(name);
  }

  public RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(name);
  }

  public RawDetection[] getRawDetections() {
    return LimelightHelpers.getRawDetections(name);
  }

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  public double getPipelineNumber() {
    return LimelightHelpers.getCurrentPipelineIndex(name);
  }

  public String getPipeline() {
    return LimelightHelpers.getCurrentPipelineType(name);
  }

  public Pose3d getPose3d() {
    return LimelightHelpers.getBotPose3d_wpiBlue(name);
  }

  public PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  }

  public PoseEstimate getPoseEstimateMegatag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }
}
