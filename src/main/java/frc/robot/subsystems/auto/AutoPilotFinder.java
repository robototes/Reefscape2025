package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.RobotConfig;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoPilotFinder extends SubsystemBase {

  // Huge thanks to 1458(Red Tie Robotics) for providing example code and help

  private static APConstraints constraints;

  static {
    try {
      var config = RobotConfig.fromGUISettings();
      constraints =
          new APConstraints()
              .withVelocity(config.moduleConfig.maxDriveVelocityMPS)
              .withAcceleration(config.moduleConfig.driveCurrentLimit)
              .withJerk(3.0);
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      constraints =
          new APConstraints()
              .withVelocity(6) // m/s
              .withAcceleration(4) // m/s²
              .withJerk(3.0);
    }
  }

  private static double kP = 2;
  private static double kI = 0;
  private static double kD = 0.2;
  private static final APProfile profile =
      new APProfile(constraints)
          .withErrorXY(Units.Centimeters.of(1))
          .withErrorTheta(Units.Degrees.of(3))
          .withBeelineRadius(Units.Centimeters.of(1));

  public static final Autopilot autoPilot =
      new Autopilot(
          profile); // autopilot insance with the desired profile based on the constraints from

  // pathplanner maybe

  public static Command createAutopilotCommand(APTarget target, CommandSwerveDrivetrain drive) {
    return drive
        .run(
            () -> {
              // Current robot speeds & pose
              ChassisSpeeds robotRelativeSpeeds = // Converted from field relative to robot relative
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      drive.getState().Speeds.vxMetersPerSecond,
                      drive.getState().Speeds.vyMetersPerSecond,
                      drive.getState().Speeds.omegaRadiansPerSecond,
                      drive.getState().Pose.getRotation());
              Pose2d pose = drive.getState().Pose;

              // Autopilot calculation
              
              APResult out =
                  autoPilot.calculate(
                      pose,
                      robotRelativeSpeeds,
                      target); // Handles calculations for going to the set point
                    

              // SwerveRequest to drive the robot to the designated point
              drive.setControl(
                  new SwerveRequest.FieldCentricFacingAngle()
                      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                      .withDriveRequestType(DriveRequestType.Velocity)
                      .withHeadingPID(kP, kI, kD)
                      .withVelocityX(out.vx())
                      .withVelocityY(out.vy())
                      .withTargetDirection(out.targetAngle()));
            })
        .withName("AutopilotCommand")
        .until(() -> autoPilot.atTarget(drive.getState().Pose, target));
  }
}
