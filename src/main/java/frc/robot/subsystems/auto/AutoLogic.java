package frc.robot.subsystems.auto;

import static frc.robot.Subsystems.SubsystemConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
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
/*import frc.team2412.robot.commands.intake.AllInCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.intake.FeederStopCommand;
import frc.team2412.robot.commands.intake.IntakeRejectCommand;
import frc.team2412.robot.commands.intake.IntakeStopCommand;
import frc.team2412.robot.commands.intake.NoteStealCommand;
import frc.team2412.robot.commands.launcher.FullTargetCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.commands.launcher.SetLaunchSpeedCommand;
import frc.team2412.robot.commands.launcher.StopLauncherCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem; */
import frc.robot.subsystems.auto.DynamicSendableChooser;
import frc.robot.subsystems.auto.PathPlannerAutos;
import frc.robot.subsystems.auto.PathPlannerAutos.Auto;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class AutoLogic {
	public static Robot r = Robot.getInstance();
	public static final Subsystems s = r.subsystems;
	public static final Controls controls = r.controls;

	public static final double FEEDER_DELAY = 0.4;

	// rpm to rev up launcher before launching
	public static final double REV_RPM = 2500;
	public static final double STAGE_ANGLE = 262;

	public static enum StartPosition {
		AMP_SIDE_SUBWOOFER(
				"Amp Side Subwoofer", new Pose2d(0.73, 6.62, new Rotation2d(Units.degreesToRadians(-120)))),
		MID_SIDE_SUBWOOFER(
				"Mid Side Subwoofer", new Pose2d(1.33, 5.55, new Rotation2d(Units.degreesToRadians(180)))),
		SOURCE_SIDE_SUBWOOFER(
				"Source Side Subwoofer",
				new Pose2d(0.73, 4.47, new Rotation2d(Units.degreesToRadians(120)))),
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
					// presets
					new AutoPath("Test Path Rotate", "5mForwardRotate180"),
					new AutoPath("Test Path", "DiameterTest"),
					new AutoPath("Master PID Test", "MasterPIDTest"),
					new AutoPath("Tune Translational PID", "TuneTranslationalPID"),
					new AutoPath("Tune Rotational PID", "TuneRotationalPID"),
					new AutoPath("Stand Still", "PresetSourceSide1Score"),
					new AutoPath("Stand Still", "PresetMid1Score"),
					new AutoPath("Stand Still", "PresetAmpSide1Score"),
					new AutoPath("Subwoofer Launch Test", "SubwooferLaunchTest"),
					// new AutoPath("Pass Auto Line", "PresetSourceSide1ScorePassAutoLine"),
					new AutoPath("Pass Autoline", "PresetAmpSide1ScorePassAutoline"),
					new AutoPath("Pass Autoline", "PresetSourceSide1ScorePassAutoline"),
					new AutoPath("Vision Launch Test", "VisionLaunchTest", true),
					new AutoPath("Steal Test", "StealTest"));

	private static List<AutoPath> onePiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1", "PresetAmpSide2Score"),
					new AutoPath("Autoline N2", "PresetMidAutoline2Score"),
					new AutoPath("Autoline N3", "PresetSourceSideAutoline2Score"),
					new AutoPath("Centerline N5", "PresetSourceSideFar2Score")
					// vision
					);

	private static List<AutoPath> twoPiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1 Centerline N1", "PresetAmpSideAutoline3Score"),
					new AutoPath("Autoline N2 N1", "PresetMidAutoline3Score"),
					new AutoPath("Autoline N2 N3", "PresetMidAutoline3Score2"),
					new AutoPath("Centerline N5 N4", "PresetSourceSideCenterline3Score2"),
					new AutoPath("Centerline N5 N3", "PresetSourceSideCenterline3Score2"),
					// vision
					new AutoPath("Centerline N5 N4", "VisionSourceSide3Score", true),
					new AutoPath("Centerline N3 N1", "VisionMidFar2Score", true),
					new AutoPath("Autoline N1 Centerline N1", "VisionAmpSideFarAutoline3Score", true));

	private static List<AutoPath> threePiecePaths =
			List.of(
					// presets
					new AutoPath("Autoline N1 N2 N3", "PresetAmpSideAutoline4Score"),
					new AutoPath("Autoline N2 N3 N1", "PresetMidAutoline4Score"),
					new AutoPath("Autoline N3 N2 N1", "PresetSourceSideAutoline4Score"),
					new AutoPath("Autoline N1 Centerline N1 Autoline N2", "PresetAmpSideAutolineFar4Score"),
					// vision
					new AutoPath("Autoline N1 Centerline N1 N2", "VisionAmpSide4Score", true),
					new AutoPath("Autoline N1 N2 N3", "VisionAmpSideAutoline4Score", true),
					new AutoPath("Autoline N3 N2 N1", "VisionMid4Score", true),
					new AutoPath("Autoline N2 Centerline N3 N1", "VisionMidFar4Score2", true),
					new AutoPath("Autoline N2 Centerline N3 N2", "VisionMidFar4Score3", true),
					new AutoPath("Autoline N3 N2 N1", "VisionSourceSideAutoline4Score", true));

	private static List<AutoPath> fourPiecePaths =
			List.of(
					// presets
					// vision
					new AutoPath("Centerline N1 Autoline N1 N2 N3", "VisionAmpSideAutoline5Score", true),
					new AutoPath("Autoline N1 Centerline N1 N2 Autoline N2", "VisionAmpSide5Score", true));

	private static List<AutoPath> fivePiecePaths =
			List.of(new AutoPath("GTA(Centerline N5N4N3N2N1)", "VisionSourceSideGrandTheftAuto", true));

	private static List<AutoPath> sixPiecePaths =
			List.of(
					new AutoPath(
							"Autoline N1 GTA(Centerline N1N2N3N4N5)", "VisionAmpSideGrandTheftAuto", true));
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
					fivePiecePaths,
					6,
					sixPiecePaths);

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

	// methods

	public static Command registerAuto(String autoName, Command command, String... primaryPathNames) {
		PathPlannerAutos.registerAuto(autoName, primaryPathNames);
		return command.withName(autoName);
	}

	/** Registers commands in PathPlanner */
	public static void registerCommands() {
		// param: String commandName, Command command

		// Intake
		//NamedCommands.registerCommand("StopIntake", stopIntake());
		//NamedCommands.registerCommand("Intake", intake());
		//NamedCommands.registerCommand("Feed", subwooferLaunch());
		//NamedCommands.registerCommand("NoteSteal", noteSteal());
		//NamedCommands.registerCommand("Reject", reject());

		// Launcher

		//NamedCommands.registerCommand("VisionLaunch", visionLaunch());
		//NamedCommands.registerCommand("SetAngleSubwoofer", setAngleSubwoofer());
		//NamedCommands.registerCommand("SubwooferLaunch", subwooferLaunch());
		//NamedCommands.registerCommand("StopLaunch", stopLaunching());
		//NamedCommands.registerCommand("RetractPivot", setAngleRetracted());

		//NamedCommands.registerCommand("RevLauncher", revFlyWheels());
	}

	// public Command getConditionalCommand(){}

	/**
	 * Takes a PathPlanner path and returns it as a command.
	 *
	 * @param pathName
	 * @return follow path command
	 */
	public static Command getAutoCommand(String pathName) {
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

	public static Optional<String> getSelectedAutoName() {
		if (availableAutos.getSelected() == null) {
			return Optional.empty();
		}
		return Optional.of(availableAutos.getSelected().getAutoName());
	}

	public static boolean chooserHasAutoSelected() {
		return availableAutos.getSelected() != null;
	}

	public static Command getSelectedAuto() {

		double waitTimer = autoDelayEntry.getDouble(0);

		return Commands.waitSeconds(waitTimer)
				.andThen(AutoBuilder.buildAuto(availableAutos.getSelected().getAutoName()));
	}

	/**
	 * Takes all of the trajectories of an auto to find the total estimated duration of an auto
	 *
	 * @return auto duration in seconds
	 */
	public static double getEstimatedAutoDuration() {
		if (getSelectedAutoName().isPresent()) {

			Auto auto = PathPlannerAutos.getAuto(getSelectedAutoName().get());
			double autoTime = 0;

			for (PathPlannerTrajectory trajectory : auto.trajectories) {
				autoTime += trajectory.getTotalTimeSeconds();
			}

			// TODO: more accurate estimating by viewing named commands involved

			// rounds time to two decimals
			autoTime *= 100;
			autoTime = ((double) ((int) autoTime)) / 100;

			// add autoDelay to estimation as well
			autoTime += autoDelayEntry.getDouble(0);

			return autoTime;
		}
		return 0;
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