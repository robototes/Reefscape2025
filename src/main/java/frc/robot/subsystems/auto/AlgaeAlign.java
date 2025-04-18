package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.util.List;

public class AlgaeAlign {
  public static Command algaeAlign(CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return new AlgaeAlignCommand(drivebaseSubsystem, controls).withName("Algae Align");
  }

  private static class AlgaeAlignCommand extends Command {
    private static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);


private static Transform2d robotOffset =   new Transform2d(Units.inchesToMeters(36.5 / 2), 0, Rotation2d.k180deg);
    private static final Pose2d blueAlgaeAB =
        aprilTagFieldLayout.getTagPose(18).get().toPose2d().plus(robotOffset);
   
    private static final Pose2d blueAlgaeCD =
        aprilTagFieldLayout.getTagPose(17).get().toPose2d().plus(robotOffset);
   
    private static final Pose2d blueAlgaeEF =
        aprilTagFieldLayout.getTagPose(22).get().toPose2d().plus(robotOffset);
 
    private static final Pose2d blueAlgaeGH =
        aprilTagFieldLayout.getTagPose(21).get().toPose2d().plus(robotOffset);
    
    private static final Pose2d blueAlgaeIJ =
        aprilTagFieldLayout.getTagPose(20).get().toPose2d().plus(robotOffset);

    private static final Pose2d blueAlgaeKL =
        aprilTagFieldLayout.getTagPose(19).get().toPose2d().plus(robotOffset);
   

    private static final Pose2d redAlgaeAB =
        aprilTagFieldLayout.getTagPose(7).get().toPose2d().plus(robotOffset);

    private static final Pose2d redAlgaeCD =
        aprilTagFieldLayout.getTagPose(8).get().toPose2d().plus(robotOffset);
   
    private static final Pose2d redAlgaeEF =
        aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(robotOffset);
  
    private static final Pose2d redAlgaeGH =
        aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(robotOffset);
 
    private static final Pose2d redAlgaeIJ =
        aprilTagFieldLayout.getTagPose(11).get().toPose2d().plus(robotOffset);
   private static final Pose2d redAlgaeKL =
        aprilTagFieldLayout.getTagPose(6).get().toPose2d().plus(robotOffset);
  

    private static final List<Pose2d> blueAlgaePoses =
        List.of(
           blueAlgaeAB,
            blueAlgaeCD,
            blueAlgaeEF,
            blueAlgaeGH,
            blueAlgaeIJ,
            blueAlgaeKL);
           
    private static final List<Pose2d> redAlgaePoses =
        List.of(
            redAlgaeAB,
            redAlgaeCD,
            redAlgaeEF,
            redAlgaeGH,
            redAlgaeIJ,
            redAlgaeKL);
           

    public static Pose2d getNearestAlgae(Pose2d p, boolean isBlue) {
      List<Pose2d> algaePose2ds = isBlue ? blueAlgaePoses : redAlgaePoses;
      return p.nearest(algaePose2ds);
    }

    private final PIDController pidX = new PIDController(4, 0, 0);
    private final PIDController pidY = new PIDController(4, 0, 0);
    private final PIDController pidRotate = new PIDController(8, 0, 0);

    private final CommandSwerveDrivetrain drive;
    private final Controls controls;
    private Pose2d algaePose;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AlgaeAlignCommand(CommandSwerveDrivetrain drive, Controls controls) {
      this.drive = drive;
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      this.controls = controls;
      setName("Algae Align");
    }

    @Override
    public void initialize() {
      boolean redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
      Pose2d robotPose = drive.getState().Pose;
      algaePose = getNearestAlgae(robotPose, !redAlliance);
      pidX.setSetpoint(algaePose.getX());
      pidY.setSetpoint(algaePose.getY());
      pidRotate.setSetpoint(algaePose.getRotation().getRadians());
    }

    @Override
    public void execute() {
      Pose2d currentPose = drive.getState().Pose;
      // Calculate the power for X direction and clamp it between -1 and 1
      double powerX = pidX.calculate(currentPose.getX());
      double powerY = pidY.calculate(currentPose.getY());
      powerX = MathUtil.clamp(powerX, -2, 2);
      powerY = MathUtil.clamp(powerY, -2, 2);
      powerX += .05 * Math.signum(powerX);
      powerY += .05 * Math.signum(powerY);
      double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
      powerRotate = MathUtil.clamp(powerRotate, -4, 4);
      SwerveRequest request =
          driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Pose2d currentPose = drive.getState().Pose;
      Transform2d robotToAlgae = algaePose.minus(currentPose);
      if (robotToAlgae.getTranslation().getNorm() < 0.01
          && Math.abs(robotToAlgae.getRotation().getDegrees()) < 1) {
        controls.vibrateDriveController(0.5);
        return true;
      }
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
      SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
      // Set the drive control with the stop request to halt all movement
      drive.setControl(stop);
      controls.vibrateDriveController(0);
    }
  }
}
