package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
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

public class AutoAlign {
  public static Command autoAlign(CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return new AutoAlignCommand(drivebaseSubsystem, controls).withName("Auto Align");
  }

  public static Command autoAlignLeft(
      CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return new AutoAlignCommandLeft(drivebaseSubsystem, controls).withName("Auto Align");
  }

  public static Command autoAlignRight(
      CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return new AutoAlignCommandRight(drivebaseSubsystem, controls).withName("Auto Align");
  }

  public static Boolean isBlue() {
    boolean isBlue;

    if (!DriverStation.getAlliance().isEmpty()) {
      isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
    } else {
      isBlue = false;
    }
    return isBlue;
  }

  public static boolean readyToScore() {
    return isStationary() && isLevel() && isCloseEnough();
  }

  public static boolean poseInPlace() {
    return isStationary() && isCloseEnough();
  }

  public static boolean isStationary() {
    var speeds = AutoLogic.s.drivebaseSubsystem.getState().Speeds;
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Units.degreesToRadians(2));
  }

  public static boolean isLevel() {
    var rotation = AutoLogic.s.drivebaseSubsystem.getRotation3d();
    return MathUtil.isNear(0, rotation.getX(), Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getY(), Units.degreesToRadians(2));
  }

  public static boolean isCloseEnough() {
    var currentPose = AutoLogic.s.drivebaseSubsystem.getState().Pose;
    var branchPose = AutoAlignCommand.getNearestBranch(currentPose);
    return currentPose.getTranslation().getDistance(branchPose.getTranslation()) < 0.05;
  }

  public static boolean
      oneSecondLeft() { // THIS WILL ONLY WORK ON THE REAL FIELD AND IN PRACTICE MODE!

    return DriverStation.getMatchTime() <= 1;
  }

  private static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // left and right offsets from the april tags ()
  private static final Transform2d leftOfTag =
      new Transform2d(
          Units.inchesToMeters(36 / 2), Units.inchesToMeters(-12.97 / 2), Rotation2d.k180deg);
  private static final Transform2d rightOfTag =
      new Transform2d(
          Units.inchesToMeters(36 / 2), Units.inchesToMeters(12.97 / 2), Rotation2d.k180deg);

  private static final Pose2d blueBranchA =
      aprilTagFieldLayout.getTagPose(18).get().toPose2d().plus(leftOfTag);
  private static final Pose2d blueBranchB =
      aprilTagFieldLayout.getTagPose(18).get().toPose2d().plus(rightOfTag);
  private static final Pose2d blueBranchC =
      aprilTagFieldLayout.getTagPose(17).get().toPose2d().plus(leftOfTag);
  private static final Pose2d blueBranchD =
      aprilTagFieldLayout.getTagPose(17).get().toPose2d().plus(rightOfTag);
  private static final Pose2d blueBranchE =
      aprilTagFieldLayout.getTagPose(22).get().toPose2d().plus(leftOfTag);
  private static final Pose2d blueBranchF =
      aprilTagFieldLayout.getTagPose(22).get().toPose2d().plus(rightOfTag);
  private static final Pose2d blueBranchG =
      aprilTagFieldLayout.getTagPose(21).get().toPose2d().plus(leftOfTag);
  private static final Pose2d blueBranchH =
      aprilTagFieldLayout.getTagPose(21).get().toPose2d().plus(rightOfTag);
  private static final Pose2d blueBranchI =
      aprilTagFieldLayout.getTagPose(20).get().toPose2d().plus(leftOfTag);
  private static final Pose2d blueBranchJ =
      aprilTagFieldLayout.getTagPose(20).get().toPose2d().plus(rightOfTag);
  private static final Pose2d blueBranchK =
      aprilTagFieldLayout.getTagPose(19).get().toPose2d().plus(leftOfTag);
  private static final Pose2d blueBranchL =
      aprilTagFieldLayout.getTagPose(19).get().toPose2d().plus(rightOfTag);

  private static final Pose2d redBranchA =
      aprilTagFieldLayout.getTagPose(7).get().toPose2d().plus(leftOfTag);
  private static final Pose2d redBranchB =
      aprilTagFieldLayout.getTagPose(7).get().toPose2d().plus(rightOfTag);
  private static final Pose2d redBranchC =
      aprilTagFieldLayout.getTagPose(8).get().toPose2d().plus(leftOfTag);
  private static final Pose2d redBranchD =
      aprilTagFieldLayout.getTagPose(8).get().toPose2d().plus(rightOfTag);
  private static final Pose2d redBranchE =
      aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(leftOfTag);
  private static final Pose2d redBranchF =
      aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(rightOfTag);
  private static final Pose2d redBranchG =
      aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(leftOfTag);
  private static final Pose2d redBranchH =
      aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(rightOfTag);
  private static final Pose2d redBranchI =
      aprilTagFieldLayout.getTagPose(11).get().toPose2d().plus(leftOfTag);
  private static final Pose2d redBranchJ =
      aprilTagFieldLayout.getTagPose(11).get().toPose2d().plus(rightOfTag);
  private static final Pose2d redBranchK =
      aprilTagFieldLayout.getTagPose(6).get().toPose2d().plus(leftOfTag);
  private static final Pose2d redBranchL =
      aprilTagFieldLayout.getTagPose(6).get().toPose2d().plus(rightOfTag);

  private static final List<Pose2d> blueBranchPoses =
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
  ;
  private static final List<Pose2d> redBranchPoses =
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

  private static class AutoAlignCommand extends Command {
    public static Pose2d getNearestBranch(Pose2d p) {
      List<Pose2d> branchPose2ds = isBlue() ? blueBranchPoses : redBranchPoses;
      return p.nearest(branchPose2ds);
    }

    protected final PIDController pidX = new PIDController(4, 0, 0);
    protected final PIDController pidY = new PIDController(4, 0, 0);
    protected final PIDController pidRotate = new PIDController(8, 0, 0);

    protected final CommandSwerveDrivetrain drive;
    protected final Controls controls;
    protected Pose2d branchPose;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AutoAlignCommand(CommandSwerveDrivetrain drive, Controls controls) {
      this.drive = drive;
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      this.controls = controls;
      setName("Auto Align");
    }

    @Override
    public void initialize() {
      Pose2d robotPose = drive.getState().Pose;
      branchPose = getNearestBranch(robotPose);
      pidX.setSetpoint(branchPose.getX());
      pidY.setSetpoint(branchPose.getY());
      pidRotate.setSetpoint(branchPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
      Pose2d currentPose = drive.getState().Pose;
      // Calculate the power for X direction and clamp it between -1 and 1
      double powerX = pidX.calculate(currentPose.getX());
      double powerY = pidY.calculate(currentPose.getY());
      powerX = MathUtil.clamp(powerX, -2, 2);
      powerY = MathUtil.clamp(powerY, -2, 2);
      powerX += .05 * Math.signum(powerX);
      powerY += .05 * Math.signum(powerY);
      double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
      powerRotate = MathUtil.clamp(powerRotate, -4, 4);
      SwerveRequest request =
          driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Pose2d currentPose = drive.getState().Pose;
      Transform2d robotToBranch = branchPose.minus(currentPose);
      if (robotToBranch.getTranslation().getNorm() < 0.01
          && Math.abs(robotToBranch.getRotation().getDegrees()) < 1) {
        controls.vibrateDriveController(0.5);
        return true;
      }
      return false;
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

  private static class AutoAlignCommandLeft extends AutoAlignCommand {
    private static final List<Pose2d> blueLeftBranchPoses =
        List.of(blueBranchA, blueBranchC, blueBranchE, blueBranchG, blueBranchI, blueBranchK);

    private static final List<Pose2d> redLeftBranchPoses =
        List.of(redBranchA, redBranchC, redBranchE, redBranchG, redBranchI, redBranchK);

    public static Pose2d getNearestLeftBranch(Pose2d p, boolean isBlue) {
      List<Pose2d> branchPose2ds = isBlue ? blueLeftBranchPoses : redLeftBranchPoses;
      return p.nearest(branchPose2ds);
    }

    public AutoAlignCommandLeft(CommandSwerveDrivetrain drive, Controls controls) {
      super(drive, controls);
    }

    @Override
    public void initialize() {
      Pose2d robotPose = drive.getState().Pose;
      branchPose = getNearestLeftBranch(robotPose, isBlue());
      pidX.setSetpoint(branchPose.getX());
      pidY.setSetpoint(branchPose.getY());
      pidRotate.setSetpoint(branchPose.getRotation().getRadians());
    }
  }

  private static class AutoAlignCommandRight extends AutoAlignCommand {
    private static final List<Pose2d> blueRightBranchPoses =
        List.of(blueBranchB, blueBranchD, blueBranchF, blueBranchH, blueBranchJ, blueBranchL);

    private static final List<Pose2d> redRightBranchPoses =
        List.of(redBranchB, redBranchD, redBranchF, redBranchH, redBranchJ, redBranchL);

    public static Pose2d getNearestRightBranch(Pose2d p) {
      List<Pose2d> branchPose2ds = isBlue() ? blueRightBranchPoses : redRightBranchPoses;
      return p.nearest(branchPose2ds);
    }

    public AutoAlignCommandRight(CommandSwerveDrivetrain drive, Controls controls) {
      super(drive, controls);
    }

    @Override
    public void initialize() {
      Pose2d robotPose = drive.getState().Pose;
      branchPose = getNearestRightBranch(robotPose);
      pidX.setSetpoint(branchPose.getX());
      pidY.setSetpoint(branchPose.getY());
      pidRotate.setSetpoint(branchPose.getRotation().getRadians());
    }
  }
}
