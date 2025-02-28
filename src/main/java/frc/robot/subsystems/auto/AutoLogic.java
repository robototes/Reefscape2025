package frc.robot.subsystems.auto;

import static frc.robot.Subsystems.SubsystemConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
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
        new Pose2d(7.187, 7.277, new Rotation2d(Units.degreesToRadians(270)))),
    ALLIANCE_SIDE_MIDDLE(
        "Alliance Side Middle",
        new Pose2d(7.187, 6.171, new Rotation2d(Units.degreesToRadians(270)))),
    MIDDLE("MIDDLE", new Pose2d(7.187, 4.044, new Rotation2d(Units.degreesToRadians(180)))),
    OPPOSITE_ALLIANCE_SIDE_WALL(
        "Alliance Side Wall", new Pose2d(7.187, 1.908, new Rotation2d(Units.degreesToRadians(90)))),
    OPPOSITE_ALLIANCE_SIDE_MIDDLE(
        "Alliance Side Middle",
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

  private static AutoPath defaultPath = new AutoPath("do nothing", "nothing");

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

  private static List<AutoPath> twoPiecePaths = List.of(new AutoPath("YSWLSF_I-J", "YSWKSF_I-J"));

  private static List<AutoPath> threePiecePaths =
      List.of(new AutoPath("YSWLSF_I-J-K", "YSWLSF_I-J-K"));

  private static List<AutoPath> fourPiecePaths =
      List.of(new AutoPath("YSWLSF_I-J-K-L", "YSWLSF_I-J-K-L", true));
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

  // vars

  // in place of launching command cause launcher doesnt exist
  public static SequentialCommandGroup vibrateControllerCommand =
      new SequentialCommandGroup(
          new InstantCommand(() -> controls.vibrateDriveController(0.5)),
          new WaitCommand(1.5),
          new InstantCommand(() -> controls.vibrateDriveController(0.0)));

  // shuffleboard
  private static ShuffleboardTab tab = Shuffleboard.getTab("Match");

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
    // NamedCommands.registerCommand("StopIntake", stopIntake());
    // NamedCommands.registerCommand("Intake", intake());
    // NamedCommands.registerCommand("Feed", subwooferLaunch());
    // NamedCommands.registerCommand("NoteSteal", noteSteal());
    // NamedCommands.registerCommand("Reject", reject());

    // Launcher

    // NamedCommands.registerCommand("VisionLaunch", visionLaunch());
    // NamedCommands.registerCommand("SetAngleSubwoofer", setAngleSubwoofer());
    // NamedCommands.registerCommand("SubwooferLaunch", subwooferLaunch());
    // NamedCommands.registerCommand("StopLaunch", stopLaunching());
    // NamedCommands.registerCommand("RetractPivot", setAngleRetracted());

    // NamedCommands.registerCommand("RevLauncher", revFlyWheels());
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

  /*
  // commands util

  public static BooleanSupplier isReadyToLaunch() {
    // return () -> (s.launcherSubsystem.isAtAngle() && s.launcherSubsystem.isAtSpeed());

    // Should be able to launch if:
    // Launcher is at the correct angle and flywheel rpm
    // Note is finished indexing (intake all in command is finished)
    return (INTAKE_ENABLED & LAUNCHER_ENABLED
        ? () ->
            (s.launcherSubsystem.isAtAngle()
                && s.launcherSubsystem.isAtSpeed()
                && !(s.intakeSubsystem.getCurrentCommand() instanceof AllInCommand))
        : () -> true);
  }

  public static BooleanSupplier untilFeederHasNoNote() {
    // decided to go from checking for note in feeder to both feeder and index in case note is still
    // indexing
    return (INTAKE_ENABLED ? () -> !s.intakeSubsystem.feederSensorHasNote() : () -> true);
  }

  // Should be able to tell if a robot has a note based off if intake is still running when checked,
  // since if note is being indexed, intake motors should've been disabled.
  public static BooleanSupplier hasNoNote() {
    return (INTAKE_ENABLED ? () -> !s.intakeSubsystem.isIntakeRunning() : () -> true);
  }

  public static BooleanSupplier hasNote() {
    return (INTAKE_ENABLED ? () -> s.intakeSubsystem.feederSensorHasNote() : () -> true);
  }

  // registered commands

  public static Command subwooferLaunch() {

    // return (LAUNCHER_ENABLED && INTAKE_ENABLED && APRILTAGS_ENABLED
    // 		? stopFeeder()
    // 				.andThen(
    // 						Commands.either(Commands.none(), index(), s.intakeSubsystem::feederSensorHasNote))
    // 				.andThen(
    // 						new SetAngleLaunchCommand(
    // 										s.launcherSubsystem,
    // 										LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
    // 										LauncherSubsystem.SUBWOOFER_AIM_ANGLE)
    // 								.until(isReadyToLaunch())
    // 								.andThen(new WaitCommand(FEEDER_DELAY))
    // 								.andThen(new FeederInCommand(s.intakeSubsystem).until(untilNoNote())))
    // 		: Commands.none());

    // Checks if the robot has a note in the subsystem, if it does, launch
    return (LAUNCHER_ENABLED && INTAKE_ENABLED && APRILTAGS_ENABLED
            ? stopFeeder()
                .andThen(
                    Commands.either(
                        new SetAngleLaunchCommand(
                                s.launcherSubsystem,
                                LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
                                LauncherSubsystem.SUBWOOFER_AIM_ANGLE)
                            .until(isReadyToLaunch())
                            .andThen(new WaitCommand(FEEDER_DELAY))
                            .andThen(
                                new FeederInCommand(s.intakeSubsystem)
                                    .until(untilFeederHasNoNote()))
                            .andThen(new WaitCommand(0.4)),
                        Commands.none(),
                        hasNote()))
            : Commands.none())
        .withName("Auto - SubwooferLaunchCommand");
  }

  public static Command visionLaunch() {
    return (LAUNCHER_ENABLED && INTAKE_ENABLED && APRILTAGS_ENABLED
            ? stopFeeder()
                .andThen(
                    Commands.either(
                        new FullTargetCommand(s.launcherSubsystem, s.drivebaseSubsystem, controls)
                            .until(isReadyToLaunch())
                            .andThen(Commands.waitUntil(hasNote()))
                            .andThen(new WaitCommand(FEEDER_DELAY))
                            .andThen(
                                new FeederInCommand(s.intakeSubsystem)
                                    .until(untilFeederHasNoNote()))
                            .andThen(new WaitCommand(0.4)),
                        Commands.none(),
                        hasNote()))
            : Commands.none())
        .withName("Auto - VisionLaunchCommand");
  }

  public static Command revFlyWheels() {
    return (LAUNCHER_ENABLED
            ? new SetLaunchSpeedCommand(s.launcherSubsystem, REV_RPM)
            : Commands.none())
        .withName("Auto - RevFlyWheelsCommand");
  }

  public static Command stopLaunching() {
    return (LAUNCHER_ENABLED ? new StopLauncherCommand(s.launcherSubsystem) : Commands.none())
        .withName("Auto - StopLauncherCommand");
  }

  public static Command setAngleRetracted() {
    return (LAUNCHER_ENABLED && INTAKE_ENABLED
            ? new SetAngleLaunchCommand(s.launcherSubsystem, 0, STAGE_ANGLE)
            : Commands.none())
        .withName("Auto - SetPivotRetractedCommand");
  }

  public static Command setAngleSubwoofer() {
    return (LAUNCHER_ENABLED
            ? Commands.waitUntil(s.intakeSubsystem::feederSensorHasNote)
                .andThen(
                    new SetAngleLaunchCommand(
                        s.launcherSubsystem,
                        LauncherSubsystem.SPEAKER_SHOOT_SPEED_RPM,
                        LauncherSubsystem.SUBWOOFER_AIM_ANGLE))
            : Commands.none())
        .withName("Auto - SetPivotSubwooferCommand");
  }

  public static Command feedUntilNoteLaunched() {
    return (INTAKE_ENABLED && LAUNCHER_ENABLED
            ? Commands.waitUntil(isReadyToLaunch())
                .andThen(Commands.waitUntil(hasNote()))
                .andThen(new WaitCommand(FEEDER_DELAY))
                .andThen(new FeederInCommand(s.intakeSubsystem).until(untilFeederHasNoNote()))
                .andThen(new WaitCommand(0.4))
            : Commands.none())
        .withName("Auto - FeedUntilNoteLaunchedCommand");
  }

  public static Command noteSteal() {
    return (INTAKE_ENABLED ? new NoteStealCommand(s.intakeSubsystem) : Commands.none())
        .withName("Auto - NoteStealCommand");
  }

  public static Command intake() {
    return (INTAKE_ENABLED ? new AllInCommand(s.intakeSubsystem, null) : Commands.none())
        .withName("Auto - IntakeAllInCommand");
  }

  public static Command stopIntake() {
    return (INTAKE_ENABLED ? new IntakeStopCommand(s.intakeSubsystem) : Commands.none())
        .withName("Auto - StopIntakeCommand");
  }

  public static Command reject() {
    return (INTAKE_ENABLED ? new IntakeRejectCommand(s.intakeSubsystem) : Commands.none())
        .withName("Auto - IntakeRejectCommand");
  }

  public static Command index() {
    return (INTAKE_ENABLED
            ? new FeederInCommand(s.intakeSubsystem).until(s.intakeSubsystem::feederSensorHasNote)
            : Commands.none())
        .withName("Auto - IndexToFeederCommand");
  }

  public static Command stopFeeder() {
    return (INTAKE_ENABLED ? new FeederStopCommand(s.intakeSubsystem) : Commands.none())
        .withName("Auto - StopFeeder");
  } */
}
