package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.List;

public class BargeAlign extends Command {
  private PIDController pidX = new PIDController(4, 0, 0);
  private PIDController pidY = new PIDController(4, 0, 0);
  private PIDController pidRotate = new PIDController(8, 0, 0);

  private CommandSwerveDrivetrain drive;
  private Pose2d blueCenterBarge = new Pose2d(7.557, 1.861, Rotation2d.kZero);
  private Pose2d redCenterBarge = new Pose2d();
  private Rectangle2d blueBargePose = new Rectangle2d(blueCenterBarge, 1, 3.683);
  private Rectangle2d redBargePose = new Rectangle2d(redCenterBarge, 1, 3.683);
  private boolean redAlliance;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric() // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  public BargeAlign(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    setName("BargeAlign");
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
  public boolean isFinished() {
    Pose2d currentPose = drive.getState().Pose;
    Transform2d robotToBranch = branchPose.minus(currentPose);
    if (robotToBranch.getTranslation().getNorm() < 0.01
        && Math.abs(robotToBranch.getRotation().getDegrees()) < 1) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
    // Set the drive control with the stop request to halt all movement
    drive.setControl(stop);
  }
}
