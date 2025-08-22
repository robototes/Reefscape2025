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

public class AutoAlignRight {
  public static Command autoAlign(CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return new AutoAlignRightCommand(drivebaseSubsystem, controls).withName("Auto Align");
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
    var branchPose = AutoAlignRightCommand.getNearestBranch(currentPose, isBlue());
    return currentPose.getTranslation().getDistance(branchPose.getTranslation()) < 0.05;
  }

  public static boolean
      oneSecondLeft() { // THIS WILL ONLY WORK ON THE REAL FIELD AND IN PRACTICE MODE!

    return DriverStation.getMatchTime() <= 1;
  }

  private static class AutoAlignRightCommand extends Command {
    private static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // left and right offsets from the april tags ()
    private static final Transform2d leftReef =
        new Transform2d(
            Units.inchesToMeters(36.5 / 2), Units.inchesToMeters(12.97 / 2), Rotation2d.k180deg);

    private static final Pose2d blueBranchB =
        aprilTagFieldLayout.getTagPose(18).get().toPose2d().plus(leftReef);
    private static final Pose2d blueBranchD =
        aprilTagFieldLayout.getTagPose(17).get().toPose2d().plus(leftReef);
    private static final Pose2d blueBranchF =
        aprilTagFieldLayout.getTagPose(22).get().toPose2d().plus(leftReef);
    private static final Pose2d blueBranchH =
        aprilTagFieldLayout.getTagPose(21).get().toPose2d().plus(leftReef);
    private static final Pose2d blueBranchJ =
        aprilTagFieldLayout.getTagPose(20).get().toPose2d().plus(leftReef);
    private static final Pose2d blueBranchL =
        aprilTagFieldLayout.getTagPose(19).get().toPose2d().plus(leftReef);

    private static final Pose2d redBranchB =
        aprilTagFieldLayout.getTagPose(7).get().toPose2d().plus(leftReef);
    private static final Pose2d redBranchD =
        aprilTagFieldLayout.getTagPose(8).get().toPose2d().plus(leftReef);
    private static final Pose2d redBranchF =
        aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(leftReef);
    private static final Pose2d redBranchH =
        aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(leftReef);
    private static final Pose2d redBranchJ =
        aprilTagFieldLayout.getTagPose(11).get().toPose2d().plus(leftReef);
    private static final Pose2d redBranchL =
        aprilTagFieldLayout.getTagPose(6).get().toPose2d().plus(leftReef);

    private static final List<Pose2d> blueBranchPoses =
        List.of(blueBranchB, blueBranchD, blueBranchF, blueBranchH, blueBranchJ, blueBranchL);
    ;
    private static final List<Pose2d> redBranchPoses =
        List.of(redBranchB, redBranchD, redBranchF, redBranchH, redBranchJ, redBranchL);

    public static Pose2d getNearestBranch(Pose2d p, boolean isBlue) {
      List<Pose2d> branchPose2ds = isBlue ? blueBranchPoses : redBranchPoses;
      return p.nearest(branchPose2ds);
    }

    private final PIDController pidX = new PIDController(4, 0, 0);
    private final PIDController pidY = new PIDController(4, 0, 0);
    private final PIDController pidRotate = new PIDController(8, 0, 0);

    private final CommandSwerveDrivetrain drive;
    private final Controls controls;
    private Pose2d branchPose;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AutoAlignRightCommand(CommandSwerveDrivetrain drive, Controls controls) {
      this.drive = drive;
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      this.controls = controls;
      setName("Auto Align Right (AAv3)");
    }

    @Override
    public void initialize() {
      boolean redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
      Pose2d robotPose = drive.getState().Pose;
      branchPose = getNearestBranch(robotPose, !redAlliance);
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
}
