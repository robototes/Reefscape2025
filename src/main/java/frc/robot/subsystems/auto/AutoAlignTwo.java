package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.List;

public class AutoAlignTwo extends Command {
  private PIDController pidX = new PIDController(6, 0, 0);
  private PIDController pidY = new PIDController(6, 0, 0);
  private PIDController pidRotate = new PIDController(8, 0, 0);

  private CommandSwerveDrivetrain drive;
  private Pose2d branchPose;
  private boolean redAlliance;
  private Controls controls;
  private static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final List<Pose2d> redAprilTags =
      Arrays.asList(
          aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(11).get().toPose2d());
  private static final List<Pose2d> blueAprilTags =
      Arrays.asList(
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(22).get().toPose2d());

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric() // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoAlignTwo(CommandSwerveDrivetrain drive, Controls controls) {
    this.drive = drive;
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    this.controls = controls;
    setName("Auto Align Two");
  }

  public void initialize() {
    redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d robotPose = drive.getState().Pose;
    branchPose = AutoAlign.getClosestBranch(robotPose);
    pidX.setSetpoint(branchPose.getX());
    pidY.setSetpoint(branchPose.getY());
    pidRotate.setSetpoint(branchPose.getRotation().getRadians());
  }

  public static Pose2d getNearestTag(Pose2d p, boolean isBlue) {
    List<Pose2d> branchesPoses = isBlue ? blueAprilTags : redAprilTags;
    return p.nearest(branchesPoses);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getState().Pose;
    Transform2d robotToBranch = branchPose.minus(currentPose);
    if (robotToBranch.getTranslation().getNorm() < 0.01
        && Math.abs(robotToBranch.getRotation().getDegrees()) < 1) {
      controls.vibrateDriveController(0.5);
      return;
    }

    // Calculate the power for X direction and clamp it between -1 and 1
    double powerX = pidX.calculate(currentPose.getX());
    double powerY = pidY.calculate(currentPose.getY());

    powerX = MathUtil.clamp(powerX, -2, 2);
    powerY = MathUtil.clamp(powerY, -2, 2);
    powerX += .05 * Math.signum(powerX);
    powerY += .05 * Math.signum(powerY);
    if (redAlliance) {
      powerX *= -1;
      powerY *= -1;
    }

    double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
    powerRotate = MathUtil.clamp(powerRotate, -4, 4);
    SwerveRequest request =
        driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
    // Set the drive control with the created request
    drive.setControl(request);
  }

  public void end(boolean interrupted) {
    // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
    // Set the drive control with the stop request to halt all movement
    drive.setControl(stop);
    controls.vibrateDriveController(0);
  }
}
