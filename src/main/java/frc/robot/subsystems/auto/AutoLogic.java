package frc.robot.subsystems.auto;



import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Subsystems;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;

public class AutoLogic {

  public static Robot r = Robot.getInstance();
  public static final Subsystems s = r.subsystems;
 

  public static enum StartPosition {
    FAR_LEFT_CAGE(
        "Far Left Cage", new Pose2d(7.187, 7.277, new Rotation2d(Units.degreesToRadians(-90)))),
    CENTER_LEFT_CAGE(
        "Center Left Cage", new Pose2d(7.187, 6.171, new Rotation2d(Units.degreesToRadians(-90)))),
    MIDDLE_MIDDLE(
        "MIDDLE MIDDLE", new Pose2d(7.187, 4.044, new Rotation2d(Units.degreesToRadians(180)))),
    CENTER_RIGHT_CAGE(
        "Center Right Cage", new Pose2d(7.187, 1.908, new Rotation2d(Units.degreesToRadians(90)))),
    FAR_RIGHT_CAGE(
        "Far Right Cage", new Pose2d(7.187, 0.811, new Rotation2d(Units.degreesToRadians(90)))),
    MISC("Misc", null);

    final String title; // for shuffleboard display
    final Pose2d startPose; // for identifying path's starting positions for filtering

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  };

  // TODO: might be a duplicate, keep until after comp

  // paths lists

  private static AutoPath defaultPath = new AutoPath("do nothing", "YSM0");

  private static List<AutoPath> noPiecePaths =
      List.of(
      
  new AutoPath("do try", "M_H"),
   new AutoPath("Gy", "MLSF_H-I")
          );

  private static List<AutoPath> onePiecePaths =
      List.of(
          );

  private static List<AutoPath> twoPiecePaths =
      List.of(
          );

  private static List<AutoPath> threePiecePaths =
      List.of(
       );

  private static List<AutoPath> fourPiecePaths =
      List.of(
        );

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
          fourPiecePaths);

  private static final Map<String, AutoPath> namesToAuto = new HashMap<>();

  static {
    for (List<AutoPath> autoPaths : commandsMap.values()) {
      for (AutoPath autoPath : autoPaths) {
        namesToAuto.put(autoPath.getDisplayName(), autoPath);
      }
    }
  }



  // shuffleboard
  private static ShuffleboardTab tab = Shuffleboard.getTab("Autos");

  private static SendableChooser<StartPosition> startPositionChooser =
      new SendableChooser<StartPosition>();
  private static DynamicSendableChooser<String> availableAutos =
      new DynamicSendableChooser<String>();
  private static SendableChooser<Integer> gameObjects = new SendableChooser<Integer>();
  private static SendableChooser<Boolean> isVision = new SendableChooser<Boolean>();

  private static GenericEntry autoDelayEntry;

  /** Registers commands in PathPlanner */
  
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

    tab.addDouble("MATCH TIME(TIMER FOR AUTO)", () -> DriverStation.getMatchTime());
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
    availableAutos.setDefaultOption(defaultPath.getDisplayName(), defaultPath.getDisplayName());

    List<AutoPath> autoCommandsList = commandsMap.get(numGameObjects);

    // filter more then add to chooser
    for (AutoPath auto : autoCommandsList) {
      if (auto.getStartPose().equals(startPositionChooser.getSelected())
          && auto.isVision() == isVision.getSelected()) {
        availableAutos.addOption(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  // get auto

  public static String getSelectedAutoName() {
    return availableAutos.getSelectedName();
  }

  public static boolean chooserHasAutoSelected() {
    return availableAutos.getSelected() != null;
  }

  public static Command getSelectedAuto() {
    double waitTimer = autoDelayEntry.getDouble(0);
    AutoPath path = namesToAuto.get(getSelectedAutoName());
    if (path == null) {
      path = defaultPath;
    }
    String autoName = path.getAutoName();

    return Commands.waitSeconds(waitTimer)
        .andThen(AutoBuilder.buildAuto(autoName))
        .withName(autoName);
  }

  // commands util
 
}
