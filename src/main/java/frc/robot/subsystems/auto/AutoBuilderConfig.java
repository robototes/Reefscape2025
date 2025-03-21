package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoBuilderConfig {
  public static Robot r = Robot.getInstance();
  public static final Subsystems s = r.subsystems;

  public static void buildAuto(CommandSwerveDrivetrain drivebase) {

    try {
      AutoBuilder.configure(
          () -> drivebase.getState().Pose, // Robot pose supplier
          (pose) ->
              drivebase.resetPose(
                  pose), // Method to reset odometry (will be called if your auto has a starting
          // pose)
          () -> drivebase.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->
              drivebase.setControl(
                  new SwerveRequest.ApplyRobotSpeeds()
                      .withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE
          // ChassisSpeeds. Also optionally outputs individual module
          // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
          RobotConfig.fromGUISettings(), // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
           
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return AutoAlign.isBlue();
            }
            return false;
          },
          s.drivebaseSubsystem // Reference to this subsystem to set requirements
          );

    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
