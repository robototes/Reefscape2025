package frc.robot.util;

import static frc.robot.Subsystems.SubsystemConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;

public class AutoLogic {
  public static Robot r = Robot.getInstance();
  public static final Subsystems s = r.subsystems;
  public static final Controls controls = r.controls;

  public static SendableChooser<String> autoPicker = new SendableChooser<String>();
  public static final double FEEDER_DELAY = 0.4;

  // rpm to rev up launcher before launching
  public static final double REV_RPM = 2500;
  public static final double STAGE_ANGLE = 262;

  public static ShuffleboardTab tab = Shuffleboard.getTab("Autos");

  public static void RunAuto(CommandSwerveDrivetrain drivebase) {

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
              return alliance.get() == DriverStation.Alliance.Red;
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

  public static Command getAutoCommand(String autoName) {
    // System.out.println("Path name: " + pathName);
    // Load the path you want to follow using its name in the GUI
    try {

      return AutoBuilder.buildAuto(autoName);

    } catch (FileVersionException e) {
      // TODO: handle exception

      DriverStation.reportError("Ooofs: " + e.getMessage(), e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    return Commands.none();
  }

  public static PathPlannerPath getPathData(String pathName) {
    // System.out.println("Path name: " + pathName);
    // Load the path you want to follow using its name in the GUI
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return path;

    } catch (IOException | ParseException | FileVersionException e) {
      // TODO: handle exception

      DriverStation.reportError("Ooofs: " + e.getMessage(), e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    return null;
  }

  public static void initShuffleBoard() {

    RunAuto(s.drivebaseSubsystem);

    addAutoOptions();

    tab.add("Auto Selector", autoPicker)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.addString("Current Selected path", () -> autoPicker.getSelected());
    if (RobotState.isAutonomous()) {
      getAutoCommand(autoPicker.getSelected());
    }
    PathPlannerPath plannerPath = getPathData(autoPicker.getSelected());

    tab.addStringArray("Path Poses: ", () -> toStringArray(plannerPath.getPathPoses()));

    tab.addStringArray("Path Waypoints: ", () -> toStringArray(plannerPath.getWaypoints()));
  }

  public static <T> String[] toStringArray(List<T> dataList) {
    System.out.println("SIZE" + dataList.size());
    String[] data = new String[dataList.size()]; // TODO FIX INFINTELY REPEATING LIST

    for (int i = 0; i < dataList.size(); i++) {

      String addedData = dataList.get(i).toString();

      data[i] = "\n" + addedData;
    }

    return data;
  }

  public static PathPlannerTrajectory makeTrajectory(
      PathPlannerPath path,
      ChassisSpeeds startingSpeeds,
      Rotation2d startingRotation,
      RobotConfig config) {

    PathPlannerTrajectory trajectory =
        new PathPlannerTrajectory(path, startingSpeeds, startingRotation, config);

    return trajectory;
  }

  public static void addAutoOptions() {
    autoPicker.setDefaultOption("DEFAULT PATH", "Test Path");

    autoPicker.addOption("PATH MID RED", "MidRed");
    autoPicker.addOption("PATH LOW RED", "LowRed");

    autoPicker.addOption("Triple L4", "Triple7");
    autoPicker.addOption("CHOREO triple L4", "New Choreo");
  }
}
