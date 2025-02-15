package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.BonkTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.RobotType;

public class Controls {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private static final int THIRD_CONTROLLER_PORT = 2;

  @SuppressWarnings("UnusedVariable")
  private final CommandXboxController driverController;

  @SuppressWarnings("UnusedVariable")
  private final CommandXboxController operatorController;

  private final CommandXboxController secretThirdController;

  @SuppressWarnings("UnusedVariable")
  private final Subsystems s;

  // Swerve stuff
  private double MaxSpeed =
      RobotType.getCurrent() == RobotType.BONK
          ? BonkTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          : CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // kSpeedAt12Volts desired top speed
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
    secretThirdController = new CommandXboxController(THIRD_CONTROLLER_PORT);
    this.s = s;
    configureDrivebaseBindings();
    configureElevatorBindings();
    configureArmPivotBindings();
    configureClimbPivotBindings();
    configureSpinnyClawBindings();
    configureElevatorLEDBindings();
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
    s.drivebaseSubsystem.applyRequest(() -> brake).ignoringDisable(true).schedule();

    // driveController.a().whileTrue(s.drivebaseSubsystem.applyRequest(() ->
    // brake));
    // driveController.b().whileTrue(s.drivebaseSubsystem.applyRequest(() ->
    // point.withModuleDirection(new Rotation2d(-driveController.getLeftY(),
    // -driveController.getLeftX()))
    // ));

    // reset the field-centric heading on left bumper press
    driverController
        .back()
        .onTrue(s.drivebaseSubsystem.runOnce(() -> s.drivebaseSubsystem.seedFieldCentric()));
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
  }

  private void configureElevatorBindings() {
    if (s.elevatorSubsystem == null) {
      return;
    }
    // Controls binding goes here
    operatorController.leftTrigger().whileTrue(s.elevatorSubsystem.goUpPower());
    operatorController.rightTrigger().whileTrue(s.elevatorSubsystem.goDownPower());
    operatorController
        .y()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.LEVEL_FOUR_POS).withName("Elevator L4"));
    operatorController
        .x()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.LEVEL_THREE_POS)
                .withName("Elevator L3"));
    operatorController
        .b()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.LEVEL_TWO_POS).withName("Elevator L2"));
    operatorController
        .a()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.LEVEL_ONE_POS).withName("Elevator L1"));
    operatorController
        .rightBumper()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.INTAKE).withName("Elevator IntakePos"));
    operatorController.povUp().whileTrue(s.elevatorSubsystem.goUp());
    operatorController.povDown().whileTrue(s.elevatorSubsystem.goDown());
    operatorController
        .leftBumper()
        .onTrue(s.elevatorSubsystem.resetPosZero().ignoringDisable(true));
  }

  private void configureArmPivotBindings() {
    if (s.armPivotSubsystem == null) {
      return;
    }

    // Arm Controls binding goes here
    // s.armPivotSubsystem.setDefaultCommand(
    // s.armPivotSubsystem
    // .startMovingVoltage(() -> Volts.of(6 * secretThirdController.getLeftY()))
    // .withName("ManuallyMoveArm"));
    secretThirdController
        .povUp()
        .onTrue(s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_L4).withName("SetArmPresetL4"));
    secretThirdController
        .povLeft()
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_L2_L3).withName("SetArmPresetL2_3"));
    secretThirdController
        .povDown()
        .onTrue(s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_UP).withName("SetArmPresetUp"));
    secretThirdController
        .povRight()
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_DOWN).withName("SetArmPresetDown"));
  }

  private void configureClimbPivotBindings() {
    if (s.climbPivotSubsystem == null) {
      return;
    }
    // Idk if this is great code or horrible code
    operatorController.start().onTrue(s.climbPivotSubsystem.toggleClimb());
  }

  private void configureSpinnyClawBindings() {
    if (s.spinnyClawSubsytem == null) {
      return;
    }
    // Claw controls bindings go here
    operatorController
        .rightBumper()
        .whileTrue(s.spinnyClawSubsytem.movingVoltage(() -> Volts.of(9)));
    operatorController
        .leftBumper()
        .whileTrue(s.spinnyClawSubsytem.movingVoltage(() -> Volts.of(-9)));
  }

  private void configureElevatorLEDBindings() {
    if (s.elevatorLEDSubsystem == null) {
      return;
    }

    // s.elevatorLEDSubsystem.setDefaultCommand(
    // s.elevatorLEDSubsystem.animate(s.elevatorLEDSubsystem.rainbowAnim));
    operatorController
        .back()
        .onTrue(s.elevatorLEDSubsystem.animate(s.elevatorLEDSubsystem.larsonAnim));
    operatorController
        .start()
        .onTrue(s.elevatorLEDSubsystem.animate(s.elevatorLEDSubsystem.rainbowAnim));
    if (s.elevatorSubsystem != null) {
      Trigger hasBeen0ed = new Trigger(s.elevatorSubsystem::getHasBeen0ed);
      Commands.waitSeconds(1)
          .andThen(
              s.elevatorLEDSubsystem.colorSet(50, 0, 0).withName("LED red").ignoringDisable(true))
          .schedule();
      hasBeen0ed.onTrue(
          s.elevatorLEDSubsystem.colorSet(0, 50, 0).withName("LED green").ignoringDisable(true));
      hasBeen0ed.onFalse(
          s.elevatorLEDSubsystem.colorSet(50, 0, 0).withName("LED red").ignoringDisable(false));
    }
  }
}
