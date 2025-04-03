package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class AutoAlignTwo extends Command {
  private PIDController pidX = new PIDController(6, 0, 0);
  private PIDController pidY = new PIDController(6, 0, 0);
  private PIDController pidRotate = new PIDController(8, 0, 0);

  private CommandSwerveDrivetrain drive;
  private Pose2d branchPose;
  private boolean redAlliance;
  
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric() // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoAlignTwo (CommandSwerveDrivetrain drive) {
    this.drive = drive;
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  public void initialize(){
    redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d robotPose = drive.getState().Pose;
    branchPose = AutoAlign.getClosestBranch(robotPose);
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
    powerX += .05*Math.signum(powerX);
    powerY += .05*Math.signum(powerY);
    if (redAlliance) {
      powerX *= -1;
      powerY *= -1;
    }

    double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
    powerRotate = MathUtil.clamp(powerRotate, -4, 4);
    SwerveRequest request = driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
    // Set the drive control with the created request
    drive.setControl(request);
  }
  public void end(boolean interrupted) {
        // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        // Set the drive control with the stop request to halt all movement
        drive.setControl(stop);
  }
}
