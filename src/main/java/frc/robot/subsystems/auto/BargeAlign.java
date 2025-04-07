package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class BargeAlign extends Command {
  private PIDController pidX = new PIDController(4, 0, 0);
  private PIDController pidRotate = new PIDController(8, 0, 0);

  private CommandSwerveDrivetrain drive;
  private static final double fieldLength = 17.548; // Welded field
  private static final double blueBlacklineX = 7.09;
  private static final double redBlacklineX = fieldLength - blueBlacklineX;
  private static final double blueBargeLineX = 7.67;
  private static final double redBargeLineX = fieldLength - blueBargeLineX;

  private final SwerveRequest.FieldCentric blackLineDriveRequest =
      new SwerveRequest.FieldCentric() // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  public static boolean atScoringXPosition(CommandSwerveDrivetrain drivebasesubsystem) {
    double robotX = drivebasesubsystem.getState().Pose.getX();
    return blueBargeLineX < robotX && robotX < redBargeLineX;
  }

  public static Command driveToBlackLine(CommandSwerveDrivetrain drivebaseSubsystem) {
    return new BargeAlign(drivebaseSubsystem).withName("Drive to Black Line");
  }

  private static final SwerveRequest.FieldCentric bargeDriveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0001)
          .withRotationalDeadband(0.0001)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private static final double xBargeDriveSpeed = 0.5;

  public static Command driveToBarge(CommandSwerveDrivetrain drivebaseSubsystem) {
    return drivebaseSubsystem
        .applyRequest(
            () -> {
              boolean onRedSide = drivebaseSubsystem.getState().Pose.getX() > fieldLength / 2;
              return bargeDriveRequest
                  .withVelocityX(onRedSide ? -xBargeDriveSpeed : xBargeDriveSpeed)
                  .withVelocityY(0)
                  .withRotationalRate(0);
            })
        .finallyDo(
            () ->
                drivebaseSubsystem.setControl(
                    bargeDriveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)))
        .withName("Drive to barge");
  }

  private BargeAlign(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
    setName("drive to black line");
  }

  @Override
  public void initialize() {
    boolean onRedSide = drive.getState().Pose.getX() > fieldLength / 2;
    double targetX = onRedSide ? redBlacklineX : blueBlacklineX;
    Rotation2d targetAngle = onRedSide ? Rotation2d.k180deg : Rotation2d.kZero;
    pidX.setSetpoint(targetX);
    pidRotate.setSetpoint(targetAngle.getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getState().Pose;
    // Calculate the power for X direction and clamp it between -1 and 1
    double powerX = pidX.calculate(currentPose.getX());
    powerX = MathUtil.clamp(powerX, -2, 2);
    powerX += .05 * Math.signum(powerX);
    double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
    powerRotate = MathUtil.clamp(powerRotate, -4, 4);
    SwerveRequest request =
        blackLineDriveRequest
            .withVelocityX(powerX)
            .withVelocityY(0)
            .withRotationalRate(powerRotate);
    // Set the drive control with the created request
    drive.setControl(request);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pidX.getError()) < 0.01
        && Math.abs(Units.radiansToDegrees(pidRotate.getError())) < 1;
  }

  @Override
  public void end(boolean interrupted) {
    // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop =
        blackLineDriveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
    // Set the drive control with the stop request to halt all movement
    drive.setControl(stop);
  }
}
