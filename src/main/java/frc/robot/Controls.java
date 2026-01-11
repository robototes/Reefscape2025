package frc.robot;


import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.BonkTunerConstants;
import frc.robot.generated.CompTunerConstants;

import frc.robot.subsystems.auto.AutoAlign;

import frc.robot.util.RobotType;

import java.util.function.BooleanSupplier;

public class Controls {
  private static final int SOLO_CONTROLLER_PORT = 0;
  private static final int DRIVER_CONTROLLER_PORT = 1;
  private static final int OPERATOR_CONTROLLER_PORT = 2;
  private static final int ARM_PIVOT_SPINNY_CLAW_CONTROLLER_PORT = 3;
  private static final int ELEVATOR_CONTROLLER_PORT = 4;
  private static final int CLIMB_TEST_CONTROLLER_PORT = 5;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final CommandXboxController armPivotSpinnyClawController;
  private final CommandXboxController elevatorTestController;
  private final CommandXboxController climbTestController;
  private final CommandXboxController soloController;

  private final Subsystems s;
  

 

  // Swerve stuff
  // setting the max speed nad other similar variables depending on which drivebase it is
  public static final double MaxSpeed =
      RobotType.getCurrent() == RobotType.BONK
          ? BonkTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          : CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MAX_ACCELERATION = 50;
  private final double MAX_ROTATION_ACCELERATION = 50;
  // kSpeedAt12Volts desired top speed
  public static double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0001)
          .withRotationalDeadband(0.0001)
          .withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final BooleanSupplier driveSlowMode;

  public Controls(Subsystems s) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    armPivotSpinnyClawController = new CommandXboxController(ARM_PIVOT_SPINNY_CLAW_CONTROLLER_PORT);
    elevatorTestController = new CommandXboxController(ELEVATOR_CONTROLLER_PORT);
    climbTestController = new CommandXboxController(CLIMB_TEST_CONTROLLER_PORT);
    soloController = new CommandXboxController(SOLO_CONTROLLER_PORT);
    this.s = s;
   
    driveSlowMode = driverController.start();
    configureDrivebaseBindings();
    
    //configureSoloControllerBindings();
    
  }

 

  private Trigger connected(CommandXboxController controller) {
    return new Trigger(() -> controller.isConnected());
  }

  // takes the X value from the joystick, and applies a deadband and input scaling
  private double getDriveX() {
    // Joystick +Y is back
    // Robot +X is forward
    double input = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1);
    double inputScale = driveSlowMode.getAsBoolean() ? 0.5 : 1;
    return input * MaxSpeed * inputScale;
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    double input = MathUtil.applyDeadband(-driverController.getLeftX(), 0.1);
    double inputScale = driveSlowMode.getAsBoolean() ? 0.5 : 1;
    return input * MaxSpeed * inputScale;
  }

  // takes the rotation value from the joystick, and applies a deadband and input scaling
  private double getDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    double input = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
    double inputScale = driveSlowMode.getAsBoolean() ? 0.5 : 1;
    return input * MaxSpeed * inputScale;
  }

  // all the current control bidings
  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // the driving command for just driving around
    s.drivebaseSubsystem.setDefaultCommand(
        // s.drivebaseSubsystem will execute this command periodically

        // applying the request to drive with the inputs
        s.drivebaseSubsystem
            .applyRequest(
                () -> {
                  boolean soloConnected = soloController.isConnected();
                  SwerveRequest.FieldCentric request =
                      drive
                          .withVelocityX(soloConnected ? getSoloDriveX() : getDriveX())
                          .withVelocityY(soloConnected ? getSoloDriveY() : getDriveY())
                          .withRotationalRate(
                              soloConnected ? getSoloDriveRotate() : getDriveRotate());
                  return request;
                })
            .withName("Drive"));

    // various former controls that were previously used and could be referenced in the future

    // operatorController
    //     .povUp()
    //     .whileTrue(
    //         s.drivebaseSubsystem
    //             .applyRequest(
    //                 () ->
    //                     drive
    //                         .withVelocityX(MetersPerSecond.of(1.0))
    //                         .withVelocityY(0)
    //                         .withRotationalRate(0))
    //             .withName("1 m/s forward"));
    // operatorController
    //     .povRight()
    //     .whileTrue(
    //         s.drivebaseSubsystem
    //             .applyRequest(
    //                 () ->
    //                     drive
    //                         .withVelocityX(MetersPerSecond.of(2.0))
    //                         .withVelocityY(0)
    //                         .withRotationalRate(0))
    //             .withName("2 m/s forward"));
    // driverController.a().whileTrue(s.drivebaseSubsystem.sysIdDynamic(Direction.kForward));
    // driverController.b().whileTrue(s.drivebaseSubsystem.sysIdDynamic(Direction.kReverse));
    // driverController.y().whileTrue(s.drivebaseSubsystem.sysIdQuasistatic(Direction.kForward));
    // driverController.x().whileTrue(s.drivebaseSubsystem.sysIdQuasistatic(Direction.kReverse));

    // driveController.a().whileTrue(s.drivebaseSubsystem.applyRequest(() ->
    // brake));
    // driveController.b().whileTrue(s.drivebaseSubsystem.applyRequest(() ->
    // point.withModuleDirection(new Rotation2d(-driveController.getLeftY(),
    // -driveController.getLeftX()))
    // ));

    // reset the field-centric heading on back button press
    driverController
        .back()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(() -> s.drivebaseSubsystem.seedFieldCentric())
                .alongWith(rumble(driverController, 0.5, Seconds.of(0.3)))
                .withName("Reset gyro"));

    // logging the telemetry
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);

    // creats a swerve button that coasts the wheels
    var swerveCoastButton =
        Shuffleboard.getTab("Controls")
            .add("Swerve Coast Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    // coast the wheels
    new Trigger(() -> swerveCoastButton.getBoolean(false))
        .whileTrue(s.drivebaseSubsystem.coastMotors());
  }

 
  

  

  

  private void configureAutoAlignBindings() {
    if (s.drivebaseSubsystem == null) {
      return;
    }
    if (s.visionSubsystem != null) {
      new Trigger(() -> s.visionSubsystem.getTimeSinceLastReading() >= 5)
          .and(RobotModeTriggers.teleop())
          .whileTrue(rumble(operatorController, 0.1, Seconds.of(10)));
    }
   // driverController  AUTO ALIGN STUFF?
    //    .rightTrigger()
    //    .and(() -> scoringMode == ScoringMode.CORAL)
    //    .and(() -> branchHeight != BranchHeight.CORAL_LEVEL_ONE)
     //   .whileTrue(AutoAlign.autoAlign(s.drivebaseSubsystem, this, AutoAlign.AlignType.RIGHTB));
   // driverController
   //     .leftTrigger()
   //     .and(() -> scoringMode == ScoringMode.CORAL)
   //     .and(() -> branchHeight != BranchHeight.CORAL_LEVEL_ONE)
    //    .whileTrue(AutoAlign.autoAlign(s.drivebaseSubsystem, this, AutoAlign.AlignType.LEFTB));
  }

 
  private Command rumble(CommandXboxController controller, double vibration, Time duration) {
    return Commands.startEnd(
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, vibration),
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(duration)
        .withName("Rumble Port " + controller.getHID().getPort());
  }

  private Command heightSelectRumble() {
    return rumble(driverController, 0.5, Seconds.of(0.3))
        .alongWith(rumble(operatorController, 0.5, Seconds.of(0.3)))
        .withName("height select rumble");
  }

  private Command scoringModeSelectRumble() {
    return rumble(driverController, 1.0, Seconds.of(0.5))
        .alongWith(rumble(operatorController, 1.0, Seconds.of(0.5)))
        .withName("height select rumble");
  }

  public void vibrateDriveController(double vibration) {
    if (!DriverStation.isAutonomous()) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, vibration);
    }
  }

  public void vibrateCoDriveController(double vibration) {
    if (!DriverStation.isAutonomous()) {
      operatorController.getHID().setRumble(RumbleType.kBothRumble, vibration);
    }
  }

  private double getJoystickInput(double input) {
    if (soloController.leftStick().getAsBoolean() || soloController.rightStick().getAsBoolean()) {
      return 0; // stop driving if either stick is pressed
    }
    // Apply a deadband to the joystick input
    double deadbandedInput = MathUtil.applyDeadband(input, 0.1);
    return deadbandedInput;
  }

  // Drive for Solo controller
  // takes the X value from the joystick, and applies a deadband and input scaling
  private double getSoloDriveX() {
    // Joystick +Y is back
    // Robot +X is forward
    return getJoystickInput(-soloController.getLeftY()) * MaxSpeed;
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getSoloDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    return getJoystickInput(-soloController.getLeftX()) * MaxSpeed;
  }

  // takes the rotation value from the joystick, and applies a deadband and input scaling
  private double getSoloDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    return getJoystickInput(-soloController.getRightX()) * MaxSpeed;
  }

  /*private void configureSoloControllerBindings() {
    // Barge + Auto align left + Select scoring mode Algae
    soloController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Command scoreCommand;
                  switch (soloScoringMode) {
                    case CORAL_IN_CLAW -> {
                      scoreCommand =
                          getCoralBranchHeightCommand(ScoringType.SOLOC_LEFT)
                              .until(
                                  () ->
                                      soloController.a().getAsBoolean()
                                          || soloController.b().getAsBoolean()
                                          || soloController.x().getAsBoolean()
                                          || soloController.y().getAsBoolean());
                    }
                    case ALGAE_IN_CLAW -> {
                      Command bargeScoreCommand =
                          BargeAlign.bargeScore(
                                  s.drivebaseSubsystem,
                                  superStructure,
                                  () -> getSoloDriveX(),
                                  () -> getSoloDriveY(),
                                  () -> getSoloDriveRotate(),
                                  soloController.rightBumper())
                              .withName("Algae score then intake");
                      scoreCommand =
                          Commands.sequence(
                              bargeScoreCommand,
                              Commands.runOnce(
                                  () -> soloScoringMode = soloScoringMode.NO_GAME_PIECE));
                    }
                    case NO_GAME_PIECE -> {
                      scoreCommand =
                          Commands.parallel(
                              Commands.runOnce(() -> intakeMode = ScoringMode.ALGAE)
                                  .alongWith(scoringModeSelectRumble())
                                  .withName("Algae Scoring Mode"),
                              AutoAlgaeHeights.autoAlgaeIntakeCommand(
                                      s.drivebaseSubsystem, superStructure, this)
                                  .until(() -> sensors.armSensor.booleanInClaw()));
                    }
                    default -> scoreCommand = Commands.none();
                  }
                  CommandScheduler.getInstance().schedule(scoreCommand);
                }));
    soloController
        .leftTrigger()
        .and(() -> soloScoringMode == soloScoringMode.CORAL_IN_CLAW)
        .and(() -> branchHeight != BranchHeight.CORAL_LEVEL_ONE)
        .whileTrue(AutoAlign.autoAlign(s.drivebaseSubsystem, this, AutoAlign.AlignType.LEFTB));
    soloController
        .leftTrigger()
        .and(() -> soloScoringMode == soloScoringMode.CORAL_IN_CLAW)
        .and(() -> branchHeight == BranchHeight.CORAL_LEVEL_ONE)
        .whileTrue(AutoAlign.autoAlign(s.drivebaseSubsystem, this, AutoAlign.AlignType.L1LB));
    // Processor + Auto align right + Select scoring mode Coral
    soloController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Command scoreCommand =
                          switch (soloScoringMode) {
                            case CORAL_IN_CLAW -> getCoralBranchHeightCommand(
                                    ScoringType.SOLOC_RIGHT)
                                .until(
                                    () ->
                                        soloController.a().getAsBoolean()
                                            || soloController.b().getAsBoolean()
                                            || soloController.x().getAsBoolean()
                                            || soloController.y().getAsBoolean());
                            case ALGAE_IN_CLAW -> Commands.sequence(
                                    superStructure.algaeProcessorScore(
                                        soloController.rightBumper()),
                                    Commands.waitSeconds(0.7),
                                    Commands.runOnce(
                                        () -> soloScoringMode = soloScoringMode.NO_GAME_PIECE))
                                .withName("Processor score");
                            case NO_GAME_PIECE -> Commands.parallel(
                                Commands.runOnce(() -> intakeMode = ScoringMode.CORAL)
                                    .alongWith(scoringModeSelectRumble())
                                    .withName("Coral Scoring Mode"),
                                superStructure.coralPreIntake(),
                                s.climbPivotSubsystem.toStow());
                          };
                      CommandScheduler.getInstance().schedule(scoreCommand);
                    })
                .withName("score"));
    soloController
        .rightTrigger()
        .and(() -> soloScoringMode == soloScoringMode.CORAL_IN_CLAW)
        .and(() -> branchHeight != BranchHeight.CORAL_LEVEL_ONE)
        .whileTrue(AutoAlign.autoAlign(s.drivebaseSubsystem, this, AutoAlign.AlignType.RIGHTB));
    soloController
        .rightTrigger()
        .and(() -> soloScoringMode == soloScoringMode.CORAL_IN_CLAW)
        .and(() -> branchHeight == BranchHeight.CORAL_LEVEL_ONE)
        .whileTrue(AutoAlign.autoAlign(s.drivebaseSubsystem, this, AutoAlign.AlignType.L1RB));
    // Ground Intake
    soloController
        .leftBumper()
        .onTrue(
            Commands.parallel(
                    Commands.runOnce(() -> intakeMode = ScoringMode.CORAL),
                    superStructure.quickGroundIntake(soloController.povUp()))
                .withName("Quick Gound intake"));
    // Scoring levels coral and algae intake heights
    soloController
        .y()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_FOUR, AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR)
                .withName("coral level 4, algae level 3-4"));
    soloController
        .x()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_THREE, AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("coral level 3, algae level 2-3"));
    soloController
        .b()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_TWO, AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("coral level 2, algae level 2-3"));
    soloController
        .a()
        .onTrue(
            selectScoringHeight(BranchHeight.CORAL_LEVEL_ONE, AlgaeIntakeHeight.ALGAE_LEVEL_GROUND)
                .withName("coral level 1, algae ground level"));
    // Zero Elevator
    soloController
        .back()
        .onTrue(
            Commands.parallel(
                    s.elevatorSubsystem.resetPosZero(),
                    rumble(soloController, 0.5, Seconds.of(0.3)))
                .ignoringDisable(true)
                .withName("Reset elevator zero"));
    // Reset gyro
    soloController
        .start()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(() -> s.drivebaseSubsystem.seedFieldCentric())
                .alongWith(rumble(soloController, 0.5, Seconds.of(0.3)))
                .withName("Reset gyro"));
    // Funnel Out
    soloController.povLeft().onTrue(s.climbPivotSubsystem.toClimbOut());
    // Funnel Climbed
    soloController.povRight().onTrue(s.climbPivotSubsystem.toClimbed());
    // Intake button
    soloController
        .povDown()
        .onTrue(
            superStructure
                .coralIntake()
                .alongWith(
                    s.elevatorLEDSubsystem != null
                        ? s.elevatorLEDSubsystem
                            .tripleBlink(255, 92, 0, "Orange - Manual Coral Intake")
                            .asProxy()
                        : Commands.none())
                .withName("Manual Coral Intake"));
    // Ground Intake Hold out
    soloController
        .povUp()
        .whileTrue(
            s.groundArm
                .moveToPosition(GroundArm.GROUND_POSITION)
                .andThen(Commands.idle())
                .withName("ground up position"));
    // Arm manual
    soloController
        .rightStick()
        .whileTrue(
            s.armPivotSubsystem
                .startMovingVoltage(() -> Volts.of(3 * soloController.getRightY()))
                .withName("Arm Manual Control"));
    // Elevator manual
    soloController
        .leftStick()
        .whileTrue(
            s.elevatorSubsystem
                .startMovingVoltage(
                    () -> Volts.of(ElevatorSubsystem.UP_VOLTAGE * -soloController.getLeftY()))
                .withName("Elevator Manual Control"));
  }*/
}
