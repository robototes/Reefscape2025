package frc.robot.util;

import static frc.robot.Subsystems.SubsystemConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
  private static final Robot r = Robot.getInstance();
  private static final Subsystems s = r.subsystems;
  private static final Controls controls = r.controls;

  private static SendableChooser<String> autoPicker = new SendableChooser<String>();
  private static SendableChooser<String> testedAutos = new SendableChooser<String>();

  public static ShuffleboardTab tab = Shuffleboard.getTab("Autos");

  public static void configureAuto(CommandSwerveDrivetrain drivebase) {

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
      DriverStation.reportError("INVALID AUTO", true);
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

    addAutoOptions();
    addTestedAutos();
    tab.add("Auto Selector", autoPicker)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.addString("Current Selected path", () -> getSelectedAutoName())
        .withPosition(0, 1)
        .withSize(2, 1);

    tab.add("Tested Autos", testedAutos)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(3, 0)
        .withSize(2, 1);
  }

  public static <T> String[] toStringArray(List<T> dataList) {

    String[] data = new String[dataList.size()];

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
    autoPicker.setDefaultOption("DEFAULT PATH", "PreLoadAuto");
    autoPicker.addOption("LOW TWO PIECE", "OppsidetoDandC");
    autoPicker.addOption("Mid 3 Piece", "Mid3Piece");
    autoPicker.addOption("Low 3 Piece", "Low3Piece");
    autoPicker.addOption("Low  1 L4", "OppsidetoD");
    autoPicker.addOption("Upper 3 Piece", "Upper3Piece");
    autoPicker.addOption("Low PreLoad", "ONE PIECE LOW");
    autoPicker.addOption("High PreLoad", "ONE PIECE HIGH");

    /* CHOREO AUTOS(IGNORE)
     autoPicker.addOption("CHOREO CRAZY TEST?(IGNORE)", "4 L4 Coral");
     autoPicker.addOption("CHOREO 3 PIECE?(IGNORE)", "New Choreo");
    autoPicker.addOption("CHOREO 3 PIECE LESS CRAZY?(IGNORE)", "Triple7");
     autoPicker.addOption("High 1 L4", "SideToJ");
     autoPicker.addOption("CHOREO High 4 L4(IGNORE)", "HighL4"); */

  }

  public static void addTestedAutos() {}

  public static Command raiseElevator() {
    if (r.superStructure != null) {
      return Commands.sequence(
          Commands.print("Pre raise elevator"),
          r.superStructure.coralLevelFour(() -> true),
          Commands.print("Pose raise elevator"));
    }
    return Commands.none().withName("raiseElevator");
  }

  public static Command autoAlignmentCommand() {
    return AutoAlign.autoAlign(s.drivebaseSubsystem).withName("autoAlign");
  }

  public static Command intakeCommand() {
    if (r.superStructure != null) {
      return r.superStructure
          .preIntake()
          .andThen(r.superStructure.coralIntake().withName("intake"));
    }
    return Commands.none().withName("intake");
  }

  public static void registerCommand() {

    if (NamedCommands.hasCommand("intake")) {
    } else {
      NamedCommands.registerCommand("intake", intakeCommand().asProxy());
    }

    if (NamedCommands.hasCommand("raiseElevator")) {
    } else {
      NamedCommands.registerCommand("raiseElevator", raiseElevator().asProxy());
    }

    if (NamedCommands.hasCommand("autoAlign")) {
    } else {
      NamedCommands.registerCommand("autoAlign", autoAlignmentCommand());
    }
  }

  public static String getSelectedAutoName() {
    return autoPicker.getSelected();
  }
}
