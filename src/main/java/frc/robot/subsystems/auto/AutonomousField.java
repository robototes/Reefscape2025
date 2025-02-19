package frc.robot.subsystems.auto;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.auto.AutoLogic;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.ObjDoubleConsumer;

public class AutonomousField {
	private static final double DEFAULT_PLAYBACK_SPEED = 2;
	private static final double UPDATE_RATE = 0.05;

	public static void configureShuffleboardTab(
			ShuffleboardTab tab,
			int columnIndex,
			int rowIndex,
			String autoChooserName,
			ObjDoubleConsumer<Runnable> addPeriodic) {
		GenericEntry speedMultiplier =
				tab.add("Auto display speed", DEFAULT_PLAYBACK_SPEED)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withProperties(Map.of("Min", 0.5, "Max", 2.5))
						.withPosition(columnIndex + 1, rowIndex + 3) // Offset by height of Field2d display
						.withSize(3, 1)
						.getEntry();

		tab.addDouble("Est. Time", AutoLogic::getEstimatedAutoDuration)
				.withPosition(columnIndex, rowIndex + 3);

		// StringSubscriber activeAutoSub =
		// 		NetworkTableInstance.getDefault()
		// 				.getTable("Shuffleboard/Match/" + autoChooserName)
		// 				.getStringTopic("active")
		// 				.subscribe("");
		var autonomousField =
				new AutonomousField(() -> speedMultiplier.getDouble(DEFAULT_PLAYBACK_SPEED));
		var watchdog =
				new Watchdog(0.001, () -> DriverStation.reportWarning("auto field loop overrun", false));
		addPeriodic.accept(
				() -> {
					watchdog.reset();

					autonomousField.update(AutoLogic.getSelectedAutoName());
					watchdog.addEpoch("AutonomousField.update()");
					watchdog.disable();
					if (watchdog.isExpired()) {
						watchdog.printEpochs();
					}
				},
				UPDATE_RATE);
		tab.add("Selected auto", autonomousField.getField())
				.withPosition(columnIndex, rowIndex)
				.withSize(4, 3);
	}

	// Display
	private final Field2d field = new Field2d();

	// Keeping track of the current trajectory
	private PathPlannerAutos.Auto auto;
	private List<PathPlannerTrajectory> trajectories;
	private int trajectoryIndex = 0;

	// Time
	private final DoubleSupplier speedMultiplier;
	private double lastFPGATime;
	private double lastTrajectoryTimeOffset;

	// Checking for changes
	private Optional<String> lastName = Optional.empty();

	public AutonomousField() {
		this(() -> 1);
	}

	public AutonomousField(double speedMultiplier) {
		this(() -> speedMultiplier);
	}

	public AutonomousField(DoubleSupplier speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	/**
	 * Returns the {@link Field2d} that this object will update.
	 *
	 * @return The Field2d object.
	 */
	public Field2d getField() {
		return field;
	}

	/**
	 * Calculates the pose to display.
	 *
	 * @param autoName The selected auto name.
	 * @return The pose along the auto trajectory
	 */
	public Pose2d getUpdatedPose(String autoName) {
		double speed = speedMultiplier.getAsDouble();
		double fpgaTime = Timer.getFPGATimestamp();
		if (lastName.isEmpty() || !lastName.get().equals(autoName)) {
			lastName = Optional.of(autoName);
			auto = PathPlannerAutos.getAuto(autoName);
			trajectories = auto.trajectories;
			trajectoryIndex = 0;
			lastFPGATime = fpgaTime;
			lastTrajectoryTimeOffset = 0;
		}
		if (trajectories.isEmpty()) {
			if (auto.startingPose != null) {
				return auto.startingPose;
			}
			return new Pose2d();
		}
		lastTrajectoryTimeOffset += (fpgaTime - lastFPGATime) * speed;
		lastFPGATime = fpgaTime;
		while (lastTrajectoryTimeOffset > trajectories.get(trajectoryIndex).getTotalTimeSeconds()) {
			lastTrajectoryTimeOffset -= trajectories.get(trajectoryIndex).getTotalTimeSeconds();
			trajectoryIndex++;
			if (trajectoryIndex >= trajectories.size()) {
				trajectoryIndex = 0;
			}
		}
		return trajectories
				.get(trajectoryIndex)
				.sample(lastTrajectoryTimeOffset)
				.getTargetHolonomicPose();
	}

	/**
	 * Updates the {@link Field2d} robot pose. If the robot is enabled, does nothing and the
	 * trajectory will restart when the robot is disabled.
	 *
	 * @param autoName The name of the selected PathPlanner autonomous routine.
	 */
	public void update(Optional<String> autoName) {
		if (DriverStation.isEnabled()) {
			lastName = Optional.empty();
			return;
		}

		if (autoName.isPresent()) {
			field.setRobotPose(getUpdatedPose(autoName.get()));
		}
	}
}