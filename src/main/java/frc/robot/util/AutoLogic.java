package frc.robot.util;

import static frc.robot.Subsystems.SubsystemConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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


import frc.robot.util.DynamicSendableChooser;
import frc.robot.util.PathPlannerAutos;
import frc.robot.util.PathPlannerAutos.Auto;

import java.io.IOException;
import java.net.PortUnreachableException;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

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
					new AutoPath("MidRed", "MidRed",false),
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

	

	 
	public static void registerCommands(String commandName, Command command) {
		
	
		NamedCommands.registerCommand(commandName, command);
		

		
		
	}
	
	// public Command getConditionalCommand(){}

	/**
	 * Takes a PathPlanner path and returns it as a command.
	 *
	 * @param pathName
	 * @return follow path command
	 * @throws org.json.simple.parser.ParseException 
	 * @throws FileVersionException 
	 */
	public static Command getAutoCommand(String pathName)   {
		// Load the path you want to follow using its name in the GUI
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
			return AutoBuilder.followPath(path);
		} catch (IOException | ParseException | FileVersionException e) {
			// TODO: handle exception
			
		}
		
		
			
		
		
		
		// Create a path following command using AutoBuilder. This will also trigger event markers.
		 return Commands.none();
	}
		 
	

	public static void initShuffleBoard() {
		
		
		
		//sab.add(availableAutos);
		
	System.out.println(getSelectedAuto());
		



/* 
		startPositionChooser.setDefaultOption(StartPosition.MISC.title, StartPosition.MISC);
		for (StartPosition startPosition : StartPosition.values()) {
			startPositionChooser.addOption(startPosition.title, startPosition);
		}
		isVision.setDefaultOption("Presets", false);
		isVision.addOption("Vision", true);
		*/

		//tab.add("Starting Position", startPositionChooser).withPosition(4, 0).withSize(2, 1);
		//tab.add("Launch Type", isVision).withPosition(4, 1);
		//tab.add("Game Objects", gameObjects).withPosition(5, 1);
		//tab.add("Available Auto Variants", availableAutos).withPosition(4, 2).withSize(2, 1);
		//autoDelayEntry = tab.add("Auto Delay", 0).withPosition(4, 3).withSize(1, 1).getEntry();

		
	}
	
	/** Takes the auto filtering entries in shuffleboard to provide a list of suitable autos */
	
	

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

	 
	
	 //* Takes all of the trajectories of an auto to find the total estimated duration of an auto
	// *
	// * @return auto duration in seconds;
	 //CHANGE LATER MAYBE????
	/* public static double getEstimatedAutoDuration() {
		if (getSelectedAutoName().isPresent()) {

		
			Auto auto = PathPlannerAuto.getPathPlannerPath((getSelectedAutoName().get()));
			
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
		
	*/

	public void RunAuto() {
		
		AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
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
            this // Reference to this subsystem to set requirements
    );
  }
}
	





	

	

