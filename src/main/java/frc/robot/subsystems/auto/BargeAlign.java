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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BargeAlign extends Command {
  private static final double fieldLength = 17.548; // Welded field
  private static final double blueBlacklineX = 7.09;
  private static final double redBlacklineX = fieldLength - blueBlacklineX;
  private static final double blueBargeLineX = 7.67;
  private static final double redBargeLineX = fieldLength - blueBargeLineX;

  private final SwerveRequest.FieldCentric blackLineDriveRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  public static boolean atScoringXPosition(CommandSwerveDrivetrain drivebasesubsystem) {
    double robotX = drivebasesubsystem.getState().Pose.getX();
    return blueBargeLineX < robotX && robotX < redBargeLineX;
  }

  private static Command driveToBlackLine(
      CommandSwerveDrivetrain drivebaseSubsystem,
      DoubleSupplier manualXSpeed,
      DoubleSupplier manualYSpeed,
      DoubleSupplier manualRotateSpeed) {
    return new BargeAlign(drivebaseSubsystem, manualXSpeed, manualYSpeed, manualRotateSpeed)
        .withName("Drive to Black Line");
  }

  private static final SwerveRequest.FieldCentric bargeDriveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0001)
          .withRotationalDeadband(0.0001)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private static final double xBargeDriveSpeed = 0.5;

  public static Command bargeScore(
      CommandSwerveDrivetrain drivebaseSubsystem,
      SuperStructure superStructure,
      DoubleSupplier manualXSpeed,
      DoubleSupplier manualYSpeed,
      DoubleSupplier manualRotateSpeed,
      BooleanSupplier manualScore) {
    return Commands.sequence(
        BargeAlign.driveToBlackLine(
                drivebaseSubsystem, manualXSpeed, manualYSpeed, manualRotateSpeed)
            .asProxy(),
        BargeAlign.driveToBarge(drivebaseSubsystem, manualXSpeed, manualYSpeed, manualRotateSpeed)
            .asProxy()
            .withDeadline(
                superStructure.algaeNetScore(
                    () ->
                        manualScore.getAsBoolean()
                            || BargeAlign.atScoringXPosition(drivebaseSubsystem))));
  }

  private static Command driveToBarge(
      CommandSwerveDrivetrain drivebaseSubsystem,
      DoubleSupplier manualXSpeed,
      DoubleSupplier manualYSpeed,
      DoubleSupplier manualRotateSpeed) {
    return drivebaseSubsystem
        .applyRequest(
            () -> {
              boolean onRedSide = drivebaseSubsystem.getState().Pose.getX() > fieldLength / 2;
              double xSpeed = onRedSide ? -xBargeDriveSpeed : xBargeDriveSpeed;
              double manualX = manualXSpeed.getAsDouble();
              if (manualX != 0) {
                xSpeed = manualX;
              }
              return bargeDriveRequest
                  .withVelocityX(xSpeed)
                  .withVelocityY(manualYSpeed.getAsDouble())
                  .withRotationalRate(manualRotateSpeed.getAsDouble());
            })
        .until(() -> BargeAlign.atScoringXPosition(drivebaseSubsystem))
        .finallyDo(
            () ->
                drivebaseSubsystem.setControl(
                    bargeDriveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)))
        .withName("Drive to barge");
  }

  private PIDController pidX = new PIDController(4, 0, 0);
  private PIDController pidRotate = new PIDController(8, 0, 0);

  private CommandSwerveDrivetrain drive;
  private DoubleSupplier manualXSpeed;
  private DoubleSupplier manualYSpeed;
  private DoubleSupplier manualRotateSpeed;

  private BargeAlign(
      CommandSwerveDrivetrain drive,
      DoubleSupplier manualXSpeed,
      DoubleSupplier manualYSpeed,
      DoubleSupplier manualRotateSpeed) {
    this.drive = drive;
    this.manualXSpeed = manualXSpeed;
    this.manualYSpeed = manualYSpeed;
    this.manualRotateSpeed = manualRotateSpeed;
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
    // Calculate the power for X direction and clamp it between -2 and 2
    double powerX = pidX.calculate(currentPose.getX());
    powerX = MathUtil.clamp(powerX, -2, 2);
    powerX += .05 * Math.signum(powerX);
    double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
    powerRotate = MathUtil.clamp(powerRotate, -4, 4);
    double manualPowerX = manualXSpeed.getAsDouble();
    if (manualPowerX != 0) {
      powerX = manualPowerX;
    }
    double manualPowerRotate = manualRotateSpeed.getAsDouble();
    if (manualPowerRotate != 0) {
      powerRotate = manualPowerRotate;
    }
    SwerveRequest request =
        blackLineDriveRequest
            .withVelocityX(powerX)
            .withVelocityY(manualYSpeed.getAsDouble())
            .withRotationalRate(powerRotate);
    // Set the drive control with the created request
    drive.setControl(request);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pidX.getError()) < 0.05
        && Math.abs(Units.radiansToDegrees(pidRotate.getError())) < 1;
  }
}
