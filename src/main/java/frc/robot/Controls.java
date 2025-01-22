package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.BonkTunerConstants;

public class Controls {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int OPERATOR_CONTROLLER_PORT = 1;

  @SuppressWarnings("UnusedVariable")
  private final CommandXboxController driverController;

  @SuppressWarnings("UnusedVariable")
  private final CommandXboxController operatorController;

  @SuppressWarnings("UnusedVariable")
  private final Subsystems s;

  // Swerve stuff
  private double MaxSpeed =
      BonkTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public Controls(Subsystems s) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    this.s = s;
    configureDrivebaseBindings();
    configureArmPivotBindings();
  }

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    s.drivebaseSubsystem.setDefaultCommand(
        // s.drivebaseSubsystem will execute this command periodically
        s.drivebaseSubsystem.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -driverController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // driveController.a().whileTrue(s.drivebaseSubsystem.applyRequest(() -> brake));
    // driveController.b().whileTrue(s.drivebaseSubsystem.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-driveController.getLeftY(),
    // -driveController.getLeftX()))
    // ));

    // reset the field-centric heading on left bumper press
    driverController
        .back()
        .onTrue(s.drivebaseSubsystem.runOnce(() -> s.drivebaseSubsystem.seedFieldCentric()));
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
  }

  private void configureArmPivotBindings() {
    if (s.armPivotSubsystem == null) {
      return;
    }
  }
}
