package frc.robot.subsystems.auto;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

/**
 * Command that automatically moves the elevator to the proper height for the nearest algae
 * based on alliance and current robot pose.
 */
public class AutoAlgaeElevatorCommand extends Command {

    private final CommandSwerveDrivetrain drivebase;
    private final ElevatorSubsystem elevator;

    private static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private static final List<Pose2d> blueAlgaePoses = List.of(
        aprilTagFieldLayout.getTagPose(17).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(19).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(21).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(18).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(20).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(22).get().toPose2d()
    );

    private static final List<Pose2d> redAlgaePoses = List.of(
        aprilTagFieldLayout.getTagPose(6).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(8).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(10).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(7).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(9).get().toPose2d(),
        aprilTagFieldLayout.getTagPose(11).get().toPose2d()
    );

    public AutoAlgaeElevatorCommand(CommandSwerveDrivetrain drivebase, ElevatorSubsystem elevator) {
        this.drivebase = drivebase;
        this.elevator = elevator;
        addRequirements(elevator); 
    }

    private static boolean isBlue() {
        return DriverStation.getAlliance()
                .map(alliance -> alliance.equals(Alliance.Blue))
                .orElse(false);
    }

    private static Pose2d getNearestAlgae(Pose2d robotPose) {
        List<Pose2d> poses = isBlue() ? blueAlgaePoses : redAlgaePoses;
        return robotPose.nearest(poses);
    }

    private static double mapAlgaePoseToElevatorHeight(Pose2d algaePose) {
        // Map poses to two heights only
        List<Pose2d> poses = isBlue() ? blueAlgaePoses : redAlgaePoses;
        int index = poses.indexOf(algaePose);

        if (index == 0 || index == 1 || index == 2) {
          return ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE; // lower reef
      } else if (index == 3 || index == 4 || index == 5) {
          return ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR; // upper reef
      } else {
          return ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE; // fallback
      }
    }

    @Override
    public void initialize() {
        Pose2d robotPose = drivebase.getState().Pose; // current robot pose
        Pose2d nearestAlgae = getNearestAlgae(robotPose);
        double targetHeight = mapAlgaePoseToElevatorHeight(nearestAlgae);

        elevator.setLevel(targetHeight).schedule(); // schedule elevator movement
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}
