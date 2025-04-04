package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
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
      List.of(
          aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(11).get().toPose2d());
  // robot dimensions - 36.5 32.5
  // reef face from april tag center 12.97/2
  private static final List<Pose2d> blueAprilTags =
      List.of(
          aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
          aprilTagFieldLayout.getTagPose(22).get().toPose2d());

  private static final List<Pose2d> blueBranchPoses;
  private static final List<Pose2d> redBranchPoses;

  // left and right offsets from the april tags ()
  private static final Transform2d leftReef =
      new Transform2d(
          Units.inchesToMeters(36.5 / 2), Units.inchesToMeters(12.97 / 2), Rotation2d.kZero);
  private static final Transform2d rightReef =
      new Transform2d(
          Units.inchesToMeters(36.5 / 2), Units.inchesToMeters(-12.97 / 2), Rotation2d.kZero);

  private static final Pose2d blueBranchA =
      aprilTagFieldLayout.getTagPose(18).get().toPose2d().plus(rightReef);
  private static final Pose2d blueBranchB =
      aprilTagFieldLayout.getTagPose(18).get().toPose2d().plus(leftReef);
  private static final Pose2d blueBranchC =
      aprilTagFieldLayout.getTagPose(17).get().toPose2d().plus(rightReef);
  private static final Pose2d blueBranchD =
      aprilTagFieldLayout.getTagPose(17).get().toPose2d().plus(leftReef);
  private static final Pose2d blueBranchE =
      aprilTagFieldLayout.getTagPose(22).get().toPose2d().plus(rightReef);
  private static final Pose2d blueBranchF =
      aprilTagFieldLayout.getTagPose(22).get().toPose2d().plus(leftReef);
  private static final Pose2d blueBranchG =
      aprilTagFieldLayout.getTagPose(21).get().toPose2d().plus(rightReef);
  private static final Pose2d blueBranchH =
      aprilTagFieldLayout.getTagPose(21).get().toPose2d().plus(leftReef);
  private static final Pose2d blueBranchI =
      aprilTagFieldLayout.getTagPose(20).get().toPose2d().plus(rightReef);
  private static final Pose2d blueBranchJ =
      aprilTagFieldLayout.getTagPose(20).get().toPose2d().plus(leftReef);
  private static final Pose2d blueBranchK =
      aprilTagFieldLayout.getTagPose(19).get().toPose2d().plus(rightReef);
  private static final Pose2d blueBranchL =
      aprilTagFieldLayout.getTagPose(19).get().toPose2d().plus(rightReef);

  private static final Pose2d redBranchA =
      aprilTagFieldLayout.getTagPose(7).get().toPose2d().plus(rightReef);
  private static final Pose2d redBranchB =
      aprilTagFieldLayout.getTagPose(7).get().toPose2d().plus(leftReef);
  private static final Pose2d redBranchC =
      aprilTagFieldLayout.getTagPose(8).get().toPose2d().plus(rightReef);
  private static final Pose2d redBranchD =
      aprilTagFieldLayout.getTagPose(8).get().toPose2d().plus(leftReef);
  private static final Pose2d redBranchE =
      aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(rightReef);
  private static final Pose2d redBranchF =
      aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(leftReef);
  private static final Pose2d redBranchG =
      aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(rightReef);
  private static final Pose2d redBranchH =
      aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(leftReef);
  private static final Pose2d redBranchI =
      aprilTagFieldLayout.getTagPose(11).get().toPose2d().plus(rightReef);
  private static final Pose2d redBranchJ =
      aprilTagFieldLayout.getTagPose(11).get().toPose2d().plus(leftReef);
  private static final Pose2d redBranchK =
      aprilTagFieldLayout.getTagPose(6).get().toPose2d().plus(rightReef);
  private static final Pose2d redBranchL =
      aprilTagFieldLayout.getTagPose(6).get().toPose2d().plus(rightReef);

  public static Pose2d getNearestTag(Pose2d p, boolean isBlue) {
    List<Pose2d> tagPose2ds = isBlue ? blueAprilTags : redAprilTags;
    return p.nearest(tagPose2ds);
  }

  public static Pose2d getNearestBranch(Pose2d p, boolean isBlue) {
    List<Pose2d> branchPose2ds = isBlue ? blueBranchPoses : redBranchPoses;
    return p.nearest(branchPose2ds);
  }

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric() // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  static {
    blueBranchPoses =
        List.of(
            blueBranchA,
            blueBranchB,
            blueBranchC,
            blueBranchD,
            blueBranchE,
            blueBranchF,
            blueBranchG,
            blueBranchH,
            blueBranchI,
            blueBranchJ,
            blueBranchK,
            blueBranchL);
    redBranchPoses =
        List.of(
            redBranchA,
            redBranchB,
            redBranchC,
            redBranchD,
            redBranchE,
            redBranchF,
            redBranchG,
            redBranchH,
            redBranchI,
            redBranchJ,
            redBranchK,
            redBranchL);
  }

  public AutoAlignTwo(CommandSwerveDrivetrain drive, Controls controls) {
    this.drive = drive;
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    this.controls = controls;
    setName("Auto Align Two");
  }

  @Override
  public void initialize() {
    redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d robotPose = drive.getState().Pose;
    branchPose = getNearestBranch(robotPose, !redAlliance);
    pidX.setSetpoint(branchPose.getX());
    pidY.setSetpoint(branchPose.getY());
    pidRotate.setSetpoint(branchPose.getRotation().getRadians());
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

  @Override
  public void end(boolean interrupted) {
    // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
    // Set the drive control with the stop request to halt all movement
    drive.setControl(stop);
    controls.vibrateDriveController(0);
  }
}
