package frc.robot.subsystems.auto;

import static frc.robot.Sensors.SensorConstants.ARMSENSOR_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;

public class AutoLogic {
  public static Robot r = Robot.getInstance();
  public static final Subsystems s = r.subsystems;
  public static final Controls controls = r.controls;

  public static final double FEEDER_DELAY = 0.4;

  // rpm to rev up launcher before launching
  public static final double REV_RPM = 2500;
  public static final double STAGE_ANGLE = 262;

  public static enum StartPosition {
    ALLIANCE_SIDE_WALL(
        "Alliance Side Wall",
        new Pose2d(7.187, 7.277, new Rotation2d(Units.degreesToRadians(-90)))),
    ALLIANCE_SIDE_MIDDLE(
        "Alliance Side Middle",
        new Pose2d(7.187, 6.171, new Rotation2d(Units.degreesToRadians(-90)))),
    MIDDLE("MIDDLE", new Pose2d(7.187, 4.044, new Rotation2d(Units.degreesToRadians(180)))),
    OPPOSITE_ALLIANCE_SIDE_MIDDLE(
        "Opposite Alliance Side Middle",
        new Pose2d(7.187, 1.908, new Rotation2d(Units.degreesToRadians(90)))),
    OPPOSITE_ALLIANCE_SIDE_WALL(
        "Opposite Alliance Side Wall",
        new Pose2d(7.187, 0.811, new Rotation2d(Units.degreesToRadians(90)))),
    MISC("Misc", null);

    final String title; // for shuffleboard display
    final Pose2d startPose; // for identifying path's starting positions for filtering

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  };

  // TODO: might be a duplicate, keep until after comp
  static {
    if (DRIVEBASE_ENABLED) {
      registerCommands();
    }
  }

  // paths lists

  private static AutoPath defaultPath = new AutoPath("do nothing", "M0");

  private static List<AutoPath> noPiecePaths =
      List.of(
          new AutoPath("YSW0", "YSW0"),
          new AutoPath("YSM0", "YSM0"),
          new AutoPath("M0", "M0"),
          new AutoPath("OSM0", "OSM0"),
          new AutoPath("OSW0", "OSW0"));

  private static List<AutoPath> onePiecePaths =
      List.of(
          new AutoPath("YSW_J", "YSW_J"),
          new AutoPath("YSW_I", "YSW_I"),
          new AutoPath("YSM_I", "YSM_I"),
          new AutoPath("M_G", "M_G"),
          new AutoPath("M_H", "M_H"),
          new AutoPath("OSM_F", "OSM_F"),
          new AutoPath("OSW_F", "OSW_F"),
          new AutoPath("OSW_E", "OSW_E"));

  private static List<AutoPath> twoPiecePaths =
      List.of(
          new AutoPath("YSWLSF_I-J", "YSWLSF_I-J"),
          new AutoPath("YSWLSF_J-K", "YSWLSF_J-K"),
          new AutoPath("YSWLSF_K-L", "YSWLSF_K-L"),
          new AutoPath("YSMLSF_I-J", "YSMLSF_I-J"),
          new AutoPath("YSMLSF_J-K", "YSMLSF_J-K"),
          new AutoPath("YSMLSF_K-L", "YSMLSF_K-L"),
          new AutoPath("OSMLSF_F-E", "OSMRSF_F-E"),
          new AutoPath("OSMLSF_E-D", "OSMRSF_E-D"),
          new AutoPath("OSMLSF_D-C", "OSMRSF_D-C"),
          new AutoPath("OSWLSF_F-E", "OSWRSF_F-E"),
          new AutoPath("OSWLSF_E-D", "OSWRSF_E-D"),
          new AutoPath("OSWLSF_D-C", "OSWRSF_D-C"));

  private static List<AutoPath> threePiecePaths =
      List.of(
          new AutoPath("YSWLSF_I-J-K", "YSWLSF_I-J-K"),
          new AutoPath("YSWLSF_J-K-L", "YSWLSF_J-K-L"),
          new AutoPath("YSWLSF_K-L-A", "YSWLSF_K-L-A"),
          new AutoPath("YSMLSF_I-J-K", "YSMLSF_I-J-K"),
          new AutoPath("YSMLSF_J-K-L", "YSMLSF_J-K-L"),
          new AutoPath("YSMLSF_K-L-A", "YSMLSF_K-L-A"),
          new AutoPath("YSWLSC_K-L-A", "YSWLSC_K-L-A"),
          new AutoPath("YSMLSC_K-L-A", "YSMLSC_K-L-A"));

  private static List<AutoPath> fourPiecePaths =
      List.of(
          new AutoPath("YSWLSF_J-K-L-A", "YSWLSF_J-K-L-A"),
          new AutoPath("YSWLSF_I-J-K-L", "YSWLSF_I-J-K-L"));

  private static List<AutoPath> testingPaths = List.of(new AutoPath("PIDTESTING", "PID TESTING"));

  // map (gulp)
  private static Map<Integer, List<AutoPath>> commandsMap =
      Map.of(
          0,
          noPiecePaths,
          1,
          onePiecePaths,
          2,
          twoPiecePaths,
          3,
          threePiecePaths,
          4,
          fourPiecePaths,
          5,
          testingPaths);

  // vars

  // in place of launching command cause launcher doesnt exist
  public static SequentialCommandGroup vibrateControllerCommand =
      new SequentialCommandGroup(
          new InstantCommand(() -> controls.vibrateDriveController(0.5)),
          new WaitCommand(1.5),
          new InstantCommand(() -> controls.vibrateDriveController(0.0)));

  // shuffleboard
  private static ShuffleboardTab tab = Shuffleboard.getTab("Autos");

  private static SendableChooser<StartPosition> startPositionChooser =
      new SendableChooser<StartPosition>();
  private static DynamicSendableChooser<AutoPath> availableAutos =
      new DynamicSendableChooser<AutoPath>();
  private static SendableChooser<Integer> gameObjects = new SendableChooser<Integer>();
  private static SendableChooser<Boolean> isVision = new SendableChooser<Boolean>();

  private static GenericEntry autoDelayEntry;

  /** Registers commands in PathPlanner */
  public static void registerCommands() {
    // param: String commandName, Command command

    // Intake
    NamedCommands.registerCommand("scoreCommand", scoreCommand());
    NamedCommands.registerCommand("branchAlign", autoBranchAlign());
    NamedCommands.registerCommand("intake", intakeCommand());
  }

  // public Command getConditionalCommand(){}

  /**
   * Takes a PathPlanner path and returns it as a command.
   *
   * @param pathName
   * @return follow path command
   * @throws ParseException
   * @throws IOException
   * @throws FileVersionException
   */
  public static Command getAutoCommand(String pathName)
      throws FileVersionException, IOException, ParseException {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }

  public static void initShuffleBoard() {
    startPositionChooser.setDefaultOption(StartPosition.MISC.title, StartPosition.MISC);
    for (StartPosition startPosition : StartPosition.values()) {
      startPositionChooser.addOption(startPosition.title, startPosition);
    }
    isVision.setDefaultOption("Presets", false);
    isVision.addOption("Vision", true);
    gameObjects.setDefaultOption("0", 0);
    for (int i = 1; i < commandsMap.size(); i++) {
      gameObjects.addOption(String.valueOf(i), i);
    }

    tab.add("Starting Position", startPositionChooser).withPosition(4, 0).withSize(2, 1);
    tab.add("Launch Type", isVision).withPosition(4, 1);
    tab.add("Game Objects", gameObjects).withPosition(5, 1);
    tab.add("Available Auto Variants", availableAutos).withPosition(4, 2).withSize(2, 1);
    autoDelayEntry = tab.add("Auto Delay", 0).withPosition(4, 3).withSize(1, 1).getEntry();

    isVision.onChange((dummyVar) -> AutoLogic.filterAutos(gameObjects.getSelected()));
    startPositionChooser.onChange((dummyVar) -> AutoLogic.filterAutos(gameObjects.getSelected()));
    gameObjects.onChange((dummyVar) -> AutoLogic.filterAutos(gameObjects.getSelected()));

    filterAutos(gameObjects.getSelected());
  }

  /** Takes the auto filtering entries in shuffleboard to provide a list of suitable autos */
  public static void filterAutos(int numGameObjects) {

    // resets/clears all options
    availableAutos.clearOptions();

    // filter based off gameobject count
    availableAutos.setDefaultOption(defaultPath.getDisplayName(), defaultPath);

    List<AutoPath> autoCommandsList = commandsMap.get(numGameObjects);

    // filter more then add to chooser
    for (AutoPath auto : autoCommandsList) {
      if (auto.getStartPose().equals(startPositionChooser.getSelected())
          && auto.isVision() == isVision.getSelected()) {
        availableAutos.addOption(auto.getDisplayName(), auto);
      }
    }
  }

  // get auto

  public static String getSelectedAutoName() {
    if (availableAutos.getSelected() == null) {
      return "nullAuto";
    }
    return availableAutos.getSelected().getAutoName();
  }

  public static boolean chooserHasAutoSelected() {
    return availableAutos.getSelected() != null;
  }

  public static Command getSelectedAuto() {

    double waitTimer = autoDelayEntry.getDouble(0);

    return Commands.waitSeconds(waitTimer)
        .andThen(AutoBuilder.buildAuto(availableAutos.getSelected().getAutoName()));
  }

  private static boolean readyToScore() {
    var speeds = s.drivebaseSubsystem.getState().Speeds;
    var rotation = s.drivebaseSubsystem.getRotation3d();
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getX(), Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getY(), Units.degreesToRadians(2));
  }

  // commands util
  public static Command scoreCommand() {
    if (r.superStructure != null) {
      return Commands.sequence(
              Commands.print("Pre raise elevator"),
              r.superStructure.coralLevelFour(() -> readyToScore()),
              Commands.print("Post raise elevator"))
          .withName("scoreCommand");
    }
    return Commands.none().withName("scoreCommand");
  }

  public static Command intakeCommand() {

    if (r.superStructure != null) {
      Command waitCommand;
      if (ARMSENSOR_ENABLED) {
        waitCommand =
            Commands.waitUntil(r.sensors.armSensor.inTrough().or(r.sensors.armSensor.inClaw()));
      } else {
        waitCommand = Commands.waitSeconds(0.5);
      }
      return Commands.sequence(
              r.superStructure.preIntake(), waitCommand, r.superStructure.coralIntake())
          .withName("intake");
    }
    return Commands.none().withName("intake");
  }

  public static Command autoBranchAlign() {
    return AutoAlign.autoAlign(s.drivebaseSubsystem).withName("autoAlign");
  }
}
