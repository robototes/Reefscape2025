package frc.robot.subsystems.auto;

import java.util.Arrays;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class AutoAlign {
	private static final double MaxAcceleration = 1;
	private static final double MaxAngularAcceleraition = 1;
	private static final Pose2d BRANCH_B_A =
			new Pose2d(new Translation2d(3.117, 4.181), Rotation2d.fromDegrees(0));
	private static final Pose2d BRANCH_B_B =
			new Pose2d(new Translation2d(3.117, 3.855), Rotation2d.fromDegrees(0));
	private static final Pose2d BRANCH_B_C =
			new Pose2d(new Translation2d(3.668, 2.910), Rotation2d.fromDegrees(60));
	private static final Pose2d BRANCH_B_D =
			new Pose2d(new Translation2d(3.920, 2.706), Rotation2d.fromDegrees(60));
	private static final Pose2d BRANCH_B_E =
			new Pose2d(new Translation2d(5.035, 2.706), Rotation2d.fromDegrees(120));	
	private static final Pose2d BRANCH_B_F =
			new Pose2d(new Translation2d(5.311, 2.910), Rotation2d.fromDegrees(120));	
	private static final Pose2d BRANCH_B_G =
			new Pose2d(new Translation2d(5.874, 3.855), Rotation2d.fromDegrees(180));
	private static final Pose2d BRANCH_B_H =
			new Pose2d(new Translation2d(5.874, 4.181), Rotation2d.fromDegrees(180));	
	private static final Pose2d BRANCH_B_I =
			new Pose2d(new Translation2d(5.311, 5.092), Rotation2d.fromDegrees(240));
	private static final Pose2d BRANCH_B_J =
			new Pose2d(new Translation2d(5.035, 5.308), Rotation2d.fromDegrees(240));
	private static final Pose2d BRANCH_B_K =
			new Pose2d(new Translation2d(5.035, 5.308), Rotation2d.fromDegrees(300));
	private static final Pose2d BRANCH_B_L =
			new Pose2d(new Translation2d(3.668, 5.092), Rotation2d.fromDegrees(300));
	private static final List<Pose2d> blueBranchesPoses = Arrays.asList(BRANCH_B_A, BRANCH_B_B, BRANCH_B_C, BRANCH_B_D, BRANCH_B_E, BRANCH_B_F, BRANCH_B_G, BRANCH_B_H, BRANCH_B_I, BRANCH_B_J, BRANCH_B_K, BRANCH_B_L);

	private static final Pose2d BRANCH_R_A =
			new Pose2d(new Translation2d(3.117, 4.181), Rotation2d.fromDegrees(0));
	private static final Pose2d BRANCH_R_B =
			new Pose2d(new Translation2d(3.117, 3.855), Rotation2d.fromDegrees(0));
	private static final Pose2d BRANCH_R_C =
			new Pose2d(new Translation2d(3.668, 2.910), Rotation2d.fromDegrees(60));
	private static final Pose2d BRANCH_R_D =
			new Pose2d(new Translation2d(3.920, 2.706), Rotation2d.fromDegrees(60));
	private static final Pose2d BRANCH_R_E =
			new Pose2d(new Translation2d(5.035, 2.706), Rotation2d.fromDegrees(120));	
	private static final Pose2d BRANCH_R_F =
			new Pose2d(new Translation2d(5.311, 2.910), Rotation2d.fromDegrees(120));	
	private static final Pose2d BRANCH_R_G =
			new Pose2d(new Translation2d(5.874, 3.855), Rotation2d.fromDegrees(180));
	private static final Pose2d BRANCH_R_H =
			new Pose2d(new Translation2d(5.874, 4.181), Rotation2d.fromDegrees(180));	
	private static final Pose2d BRANCH_R_I =
			new Pose2d(new Translation2d(5.311, 5.092), Rotation2d.fromDegrees(240));
	private static final Pose2d BRANCH_R_J =
			new Pose2d(new Translation2d(5.035, 5.308), Rotation2d.fromDegrees(240));
	private static final Pose2d BRANCH_R_K =
			new Pose2d(new Translation2d(5.035, 5.308), Rotation2d.fromDegrees(300));
	private static final Pose2d BRANCH_R_L =
			new Pose2d(new Translation2d(3.668, 5.092), Rotation2d.fromDegrees(300));
	private static final List<Pose2d> redBranchesPoses = Arrays.asList(BRANCH_R_A, BRANCH_R_B, BRANCH_R_C, BRANCH_R_D, BRANCH_R_E, BRANCH_R_F, BRANCH_R_G, BRANCH_R_H, BRANCH_R_I, BRANCH_R_J, BRANCH_R_K, BRANCH_R_L);

	public static Command autoAlign(
		CommandSwerveDrivetrain drivebaseSubsystem) {

		Pose2d robotPose = drivebaseSubsystem.getState().Pose;
		boolean isBlue;
		if (!DriverStation.getAlliance().isEmpty()) {
			isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
		} else {
			isBlue = false;
		}
		// figures out which branch to go to
		List<Pose2d> branchesPoses = isBlue ? blueBranchesPoses : redBranchesPoses;
		Pose2d branchPose = robotPose.nearest(branchesPoses);
		// sets the point for the path to go to
		List<Waypoint> waypointsPoeses = PathPlannerPath.waypointsFromPoses(robotPose, branchPose);
		// creates path
		PathPlannerPath path =
				new PathPlannerPath(
						waypointsPoeses,
						new PathConstraints(
								Controls.MaxSpeed,
								MaxAcceleration,
								Controls.MaxAngularRate,
								MaxAngularAcceleraition),
						null, new GoalEndState(0.0, branchPose.getRotation()));
		// path.flipPath(); Returns path except it's flipped
		// this unflips it
		if (!isBlue) {
			path = path.flipPath();
		}

		return AutoBuilder.followPath(path);
	}

	public static class AlignCommand extends Command {
		private final CommandSwerveDrivetrain drivebaseSubsystem;
		private Command alignCommand = null;

		public AlignCommand(
			CommandSwerveDrivetrain drivebaseSubsystem) {
			this.drivebaseSubsystem = drivebaseSubsystem;
			addRequirements(drivebaseSubsystem);
		}

		@Override
		public void initialize() {
			alignCommand = autoAlign(drivebaseSubsystem);

			alignCommand.initialize();
			// launcherSubsystem.setAngle(LauncherSubsystem.TRAP_AIM_ANGLE);
			// launcherSubsystem.launch(LauncherSubsystem.TRAP_SHOOT_SPEED_RPM);
		}

		@Override
		public void execute() {
			alignCommand.execute();
		}

		@Override
		public boolean isFinished() {
			return alignCommand.isFinished();
		}

		@Override
		public void end(boolean interrupted) {
			if (interrupted && alignCommand != null) {
				alignCommand.end(true);
			}
			alignCommand = null;
		}
	}
}