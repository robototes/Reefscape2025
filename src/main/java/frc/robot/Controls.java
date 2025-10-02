package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
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
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundArm;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.auto.AutoAlgaeHeights;
import frc.robot.subsystems.auto.AutoAlign;
import frc.robot.subsystems.auto.BargeAlign;
import frc.robot.util.AlgaeIntakeHeight;
import frc.robot.util.BranchHeight;
import frc.robot.util.RobotType;
import frc.robot.util.ScoringMode;
import frc.robot.util.SoloScoringMode;

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
  private final Sensors sensors;
  private final SuperStructure superStructure;

  private BranchHeight branchHeight = BranchHeight.CORAL_LEVEL_FOUR;
  private ScoringMode scoringMode = ScoringMode.CORAL;
  private ScoringMode intakeMode = ScoringMode.CORAL;
  private SoloScoringMode soloScoringMode = SoloScoringMode.NO_GAME_PIECE;
  private AlgaeIntakeHeight algaeIntakeHeight = AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR;

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
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final BooleanSupplier driveSlowMode;

  public Controls(Subsystems s, Sensors sensors, SuperStructure superStructure) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    armPivotSpinnyClawController = new CommandXboxController(ARM_PIVOT_SPINNY_CLAW_CONTROLLER_PORT);
    elevatorTestController = new CommandXboxController(ELEVATOR_CONTROLLER_PORT);
    climbTestController = new CommandXboxController(CLIMB_TEST_CONTROLLER_PORT);
    soloController = new CommandXboxController(SOLO_CONTROLLER_PORT);
    this.s = s;
    this.sensors = sensors;
    this.superStructure = superStructure;
    driveSlowMode = driverController.start();
    configureDrivebaseBindings();
    configureSuperStructureBindings();
    configureElevatorBindings();
    configureArmPivotBindings();
    configureClimbPivotBindings();
    configureSpinnyClawBindings();
    configureElevatorLEDBindings();
    configureAutoAlignBindings();
    configureGroundSpinnyBindings();
    configureGroundArmBindings();
    configureSoloControllerBindings();
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
                () ->
                    drive
                        .withVelocityX(soloController.isConnected() ? getSoloDriveX() : getDriveX())
                        .withVelocityY(soloController.isConnected() ? getSoloDriveY() : getDriveY())
                        .withRotationalRate(
                            soloController.isConnected() ? getSoloDriveRotate() : getDriveRotate()))
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

  private void configureSuperStructureBindings() {
    if (superStructure == null) {
      return;
    }
    superStructure.setBranchHeightSupplier(() -> branchHeight);
    // operator start button used for climb - bound in climb bindings
    operatorController
        .y()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_FOUR, AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR)
                .withName("coral level 4, algae level 3-4"));
    operatorController
        .x()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_THREE, AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("coral level 3, algae level 2-3"));
    operatorController
        .b()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_TWO, AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("coral level 2, algae level 2-3"));
    operatorController
        .a()
        .onTrue(
            selectScoringHeight(BranchHeight.CORAL_LEVEL_ONE, AlgaeIntakeHeight.ALGAE_LEVEL_GROUND)
                .withName("coral level 1, algae ground level"));
    driverController
        .povUp()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_FOUR, AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR)
                .withName("coral level 4, algae level 3-4"));
    driverController
        .povLeft()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_THREE, AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("coral level 3, algae level 2-3"));
    driverController
        .povRight()
        .onTrue(
            selectScoringHeight(
                    BranchHeight.CORAL_LEVEL_TWO, AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("coral level 2, algae level 2-3"));
    driverController
        .povDown()
        .onTrue(
            selectScoringHeight(BranchHeight.CORAL_LEVEL_ONE, AlgaeIntakeHeight.ALGAE_LEVEL_GROUND)
                .withName("coral level 1, algae ground level"));
    driverController
        .leftTrigger()
        .onTrue(
            Commands.deferredProxy(
                    () ->
                        switch (scoringMode) {
                          case CORAL -> getCoralBranchHeightCommand();
                          case ALGAE -> Commands.sequence(
                                  superStructure.algaeProcessorScore(
                                      driverController.rightBumper()),
                                  Commands.waitSeconds(0.7),
                                  getAlgaeIntakeCommand())
                              .withName("Processor score");
                        })
                .withName("Schedule processor score"));

    operatorController
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> scoringMode = ScoringMode.ALGAE)
                .alongWith(scoringModeSelectRumble())
                .withName("Algae Scoring Mode"))
        .onTrue(
            Commands.runOnce(() -> CommandScheduler.getInstance().schedule(getAlgaeIntakeCommand()))
                .withName("run algae intake"));
    operatorController // should work???
        .leftTrigger()
        .onTrue(
            Commands.runOnce(() -> scoringMode = ScoringMode.CORAL)
                .alongWith(scoringModeSelectRumble())
                .withName("Coral Scoring Mode"))
        .onTrue(superStructure.coralPreIntake())
        .onTrue(s.climbPivotSubsystem.toStow());
    operatorController
        .povLeft()
        .onTrue(
            Commands.deferredProxy(
                    () ->
                        switch (scoringMode) {
                          case CORAL -> superStructure.coralStow();
                          case ALGAE -> superStructure.algaeStow();
                        })
                .withName("Stow"));
    operatorController
        .povDown()
        .onTrue(
            Commands.deferredProxy(
                    () ->
                        switch (scoringMode) {
                          case CORAL -> superStructure.coralPreIntake();
                          case ALGAE -> superStructure.algaeStow();
                        })
                .withName("pre-intake, algae stow"));

    driverController
        .a()
        .onTrue(s.elevatorSubsystem.runOnce(() -> {}).withName("elevator interruptor"))
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Command intakeCommand =
                          switch (scoringMode) {
                            case CORAL -> superStructure
                                .coralIntake()
                                .alongWith(
                                    s.elevatorLEDSubsystem != null
                                        ? s.elevatorLEDSubsystem
                                            .tripleBlink(255, 92, 0, "Orange - Manual Coral Intake")
                                            .asProxy()
                                        : Commands.none())
                                .withName("Manual Coral Intake");
                            case ALGAE -> getAlgaeIntakeCommand();
                          };
                      CommandScheduler.getInstance().schedule(intakeCommand);
                    })
                .withName("Driver Intake"));

    driverController
        .b()
        .onTrue(
            superStructure.quickGroundIntake(driverController.x()).withName("Quick Gound intake"));

    if (sensors.armSensor != null) {
      sensors
          .armSensor
          .inTrough()
          .and(superStructure.inCoralPreIntakePosition())
          .and(RobotModeTriggers.teleop())
          .onTrue(
              superStructure
                  .coralIntake()
                  .alongWith(
                      s.elevatorLEDSubsystem != null
                          ? s.elevatorLEDSubsystem
                              .tripleBlink(255, 255, 0, "Yellow - Automatic Intake")
                              .asProxy()
                          : Commands.none())
                  .withName("Automatic Intake"));
    }

    if (sensors.armSensor != null) {
      sensors
          .armSensor
          .inClaw()
          .and(RobotModeTriggers.teleop())
          .onTrue(
              Commands.runOnce(
                      () -> {
                        switch (intakeMode) {
                          case CORAL -> soloScoringMode = SoloScoringMode.CORAL_IN_CLAW;
                          case ALGAE -> soloScoringMode = SoloScoringMode.ALGAE_IN_CLAW;
                        }
                      })
                  .withName("Set solo scoring mode"));

      sensors
          .armSensor
          .inClaw()
          .and(RobotModeTriggers.teleop())
          .onFalse(
              Commands.runOnce(
                      () -> {
                        soloScoringMode = SoloScoringMode.NO_GAME_PIECE;
                      })
                  .withName("Clear solo scoring mode"));
    }

    if (s.climbPivotSubsystem.sensor != null) {
      s.climbPivotSubsystem
          .cageDetected()
          .and(new Trigger(() -> s.climbPivotSubsystem.isClimbOut))
          .and(RobotModeTriggers.teleop())
          .onTrue(s.climbPivotSubsystem.toClimbed().withName("Automatic sensor Climbing"));
    }

    driverController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Command scoreCommand =
                          switch (scoringMode) {
                            case CORAL -> getCoralBranchHeightCommand();
                            case ALGAE -> Commands.sequence(
                                    BargeAlign.bargeScore(
                                        s.drivebaseSubsystem,
                                        superStructure,
                                        () -> getDriveX(),
                                        () -> getDriveY(),
                                        () -> getDriveRotate(),
                                        driverController.rightBumper()),
                                    getAlgaeIntakeCommand())
                                .withName("Algae score then intake");
                          };
                      CommandScheduler.getInstance().schedule(scoreCommand);
                    })
                .withName("score"));
  }

  private Command getAlgaeIntakeCommand() {
    return switch (algaeIntakeHeight) {
      case ALGAE_LEVEL_THREE_FOUR -> superStructure.algaeLevelThreeFourIntake();
      case ALGAE_LEVEL_TWO_THREE -> superStructure.algaeLevelTwoThreeIntake();
      case ALGAE_LEVEL_GROUND -> superStructure.algaeGroundIntake();
    };
  }

  private Command getCoralBranchHeightCommand() {
    return switch (branchHeight) {
      case CORAL_LEVEL_FOUR -> superStructure.coralLevelFour(driverController.rightBumper());
      case CORAL_LEVEL_THREE -> superStructure.coralLevelThree(driverController.rightBumper());
      case CORAL_LEVEL_TWO -> superStructure.coralLevelTwo(driverController.rightBumper());
      case CORAL_LEVEL_ONE -> superStructure.coralLevelOne(driverController.rightBumper());
    };
  }

  private Command getSoloCoralBranchHeightCommand() {
    return switch (branchHeight) {
      case CORAL_LEVEL_FOUR -> superStructure
          .coralLevelFour(soloController.rightBumper())
          .andThen(() -> soloScoringMode = soloScoringMode.NO_GAME_PIECE);
      case CORAL_LEVEL_THREE -> superStructure
          .coralLevelThree(soloController.rightBumper())
          .andThen(() -> soloScoringMode = soloScoringMode.NO_GAME_PIECE);
      case CORAL_LEVEL_TWO -> superStructure
          .coralLevelTwo(soloController.rightBumper())
          .andThen(() -> soloScoringMode = soloScoringMode.NO_GAME_PIECE);
      case CORAL_LEVEL_ONE -> superStructure
          .coralLevelOne(soloController.rightBumper())
          .andThen(() -> soloScoringMode = soloScoringMode.NO_GAME_PIECE);
    };
  }

  private void configureElevatorBindings() {
    if (s.elevatorSubsystem == null) {
      return;
    }
    RobotModeTriggers.disabled().onTrue(s.elevatorSubsystem.stop());
    // Controls binding goes here
    operatorController
        .leftStick()
        .whileTrue(
            s.elevatorSubsystem
                .startMovingVoltage(
                    () -> Volts.of(ElevatorSubsystem.UP_VOLTAGE * -operatorController.getLeftY()))
                .withName("Elevator Manual Control"));
    s.elevatorSubsystem.setRumble(
        (rumble) -> {
          elevatorTestController.setRumble(RumbleType.kBothRumble, rumble);
          operatorController.setRumble(RumbleType.kBothRumble, rumble);
        });
    connected(elevatorTestController)
        .and(elevatorTestController.y())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_POS)
                .withName("Elevator L4"));
    connected(elevatorTestController)
        .and(elevatorTestController.x())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_POS)
                .withName("Elevator L3"));
    connected(elevatorTestController)
        .and(elevatorTestController.b())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_POS)
                .withName("Elevator L2"));
    connected(elevatorTestController)
        .and(elevatorTestController.a())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_ONE_POS)
                .withName("Elevator L1"));
    connected(elevatorTestController)
        .and(elevatorTestController.rightBumper())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_INTAKE_POS)
                .withName("Elevator IntakePos"));
    connected(elevatorTestController)
        .and(elevatorTestController.povUp())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)
                .withName("Elevator Algae L3-L4"));
    connected(elevatorTestController)
        .and(elevatorTestController.povLeft())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)
                .withName("Elevator Algae L2-L3"));
    connected(elevatorTestController)
        .and(elevatorTestController.povRight())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_STOWED)
                .withName("Elevator Algae Stowed"));
    connected(elevatorTestController)
        .and(elevatorTestController.povDown())
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE)
                .withName("Elevator Processor"));
    connected(elevatorTestController)
        .and(elevatorTestController.leftBumper())
        .whileTrue(s.elevatorSubsystem.holdCoastMode().withName("elevatortest hold coast mode"));
    operatorController
        .back()
        .onTrue(
            Commands.parallel(
                    s.elevatorSubsystem.resetPosZero(),
                    rumble(operatorController, 0.5, Seconds.of(0.3)))
                .ignoringDisable(true)
                .withName("Reset elevator zero"));
    if (RobotBase.isSimulation()) {
      s.elevatorSubsystem
          .resetPosZero()
          .ignoringDisable(true)
          .withName("Sim - reset elevator zero")
          .schedule();
    }

    // operatorController.rightBumper().whileTrue(s.elevatorSubsystem.holdCoastMode());
    var elevatorCoastButton =
        Shuffleboard.getTab("Controls")
            .add("Elevator Coast Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    new Trigger(() -> elevatorCoastButton.getBoolean(false))
        .whileTrue(s.elevatorSubsystem.holdCoastMode());
    // var elevatorZeroButton = new DigitalInput(Hardware.ELEVATOR_ZERO_BUTTON);
    // new Trigger(() -> elevatorZeroButton.get())
    //     .debounce(1, DebounceType.kRising)
    //     .and(RobotModeTriggers.disabled())
    //     .onTrue(s.elevatorSubsystem.resetPosZero());
  }

  private void configureArmPivotBindings() {
    if (s.armPivotSubsystem == null) {
      return;
    }

    // Arm Controls binding goes here
    operatorController
        .rightStick()
        .whileTrue(
            s.armPivotSubsystem
                .startMovingVoltage(() -> Volts.of(3 * operatorController.getRightY()))
                .withName("Arm Manual Control"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.rightStick())
        .whileTrue(
            s.armPivotSubsystem
                .startMovingVoltage(() -> Volts.of(3 * armPivotSpinnyClawController.getRightY()))
                .withName("Arm Manual Control"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.povRight())
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.CORAL_PRESET_L4).withName("Arm L4 Preset"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.povLeft())
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.CORAL_PRESET_L3).withName("Arm L3 Preset"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.povUp())
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.CORAL_PRESET_UP).withName("Arm Preset Up"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.povDown())
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.CORAL_PRESET_DOWN)
                .withName("Arm Preset Down"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.y())
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.ALGAE_REMOVE)
                .withName("Algae Preset Remove"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.b())
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE)
                .withName("Algae Preset Score"));
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.a())
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.ALGAE_STOWED)
                .withName("Algae Preset Stowed"));
  }

  private void configureClimbPivotBindings() {
    if (s.climbPivotSubsystem == null) {
      return;
    }

    if (s.elevatorLEDSubsystem != null) {
      Command setClimbLEDs = s.elevatorLEDSubsystem.pulse(0, 0, 255, "Blue - Climb Extended");
      s.climbPivotSubsystem.isClimbing().whileTrue(setClimbLEDs);
    }

    // regularly run the advanced climb check
    s.climbPivotSubsystem.setDefaultCommand(
        s.climbPivotSubsystem.advanceClimbCheck().withName("Advance Climb Check"));

    // check if the climb controller is connected, and whne start is pressed move to the next climb
    // position
    connected(climbTestController)
        .and(climbTestController.start())
        .onTrue(s.climbPivotSubsystem.advanceClimbTarget());
    // on start or right trigger, move to climb out or climbed respectively on operator
    operatorController.start().onTrue(s.climbPivotSubsystem.toClimbOut());
    operatorController.rightTrigger().onTrue(s.climbPivotSubsystem.toClimbed());
    // manual control for climb test controller for negative direction
    connected(climbTestController)
        .and(climbTestController.rightTrigger(0.1))
        .whileTrue(
            s.climbPivotSubsystem
                .moveClimbManual(
                    () ->
                        -0.6
                            * MathUtil.applyDeadband(
                                climbTestController.getRightTriggerAxis(), 0.1))
                .withName("Climb Manual Control"));
    // manual control for climb test controller for positive direction
    connected(climbTestController)
        .and(climbTestController.leftTrigger(0.1))
        .whileTrue(
            s.climbPivotSubsystem
                .moveClimbManual(
                    () ->
                        0.6 * MathUtil.applyDeadband(climbTestController.getLeftTriggerAxis(), 0.1))
                .withName("Climb Manual Control"));

    // climb coast button
    var climbCoastButton =
        Shuffleboard.getTab("Controls")
            .add("Climb Coast Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    // utilizes the climb coast button and coasts the climb
    new Trigger(() -> climbCoastButton.getBoolean(false))
        .whileTrue(s.climbPivotSubsystem.coastMotors());
  }

  private void configureSpinnyClawBindings() {
    if (s.spinnyClawSubsytem == null) {
      return;
    }
    s.spinnyClawSubsytem.setScoringMode(() -> scoringMode);
    // Claw controls bindings go here
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.leftBumper())
        .whileTrue(s.spinnyClawSubsytem.coralHoldExtakePower());
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.rightBumper())
        .whileTrue(s.spinnyClawSubsytem.coralHoldIntakePower());
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.leftTrigger())
        .whileTrue(s.spinnyClawSubsytem.algaeHoldExtakePower());
    connected(armPivotSpinnyClawController)
        .and(armPivotSpinnyClawController.rightTrigger())
        .whileTrue(s.spinnyClawSubsytem.algaeGripIntakePower());
    driverController
        .rightBumper()
        .whileTrue(
            Commands.deferredProxy(
                () -> {
                  if (s.spinnyClawSubsytem.getCurrentCommand() != null) {
                    return Commands.none().withName("No manual spit - Spinny claw used");
                  } else {
                    return switch (scoringMode) {
                      case CORAL -> Commands.none().withName("No manual spit - Coral mode");
                      case ALGAE -> s.spinnyClawSubsytem.algaeExtakePower();
                    };
                  }
                }));
  }

  private void configureElevatorLEDBindings() {
    if (s.elevatorLEDSubsystem == null) {
      return;
    }

    s.elevatorLEDSubsystem.setDefaultCommand(
        s.elevatorLEDSubsystem.showScoringMode(() -> soloScoringMode));

    if (s.elevatorSubsystem != null) {
      Trigger hasBeenZeroed = new Trigger(s.elevatorSubsystem::getHasBeenZeroed);
      Trigger subZeroPosition = new Trigger(s.elevatorSubsystem::getPositionSubZero);
      Commands.waitSeconds(1)
          .andThen(
              s.elevatorLEDSubsystem
                  .blink(120, 0, 0, "Red - Elevator Not Zeroed")
                  .ignoringDisable(true))
          .schedule();
      hasBeenZeroed.onTrue(
          s.elevatorLEDSubsystem
              .colorSet(0, 170, 0, "Green - Elevator Zeroed")
              .withTimeout(2)
              .andThen(s.elevatorLEDSubsystem.colorSet(0, 32, 0, "dimmed Green - Elevator Zeroed"))
              .ignoringDisable(true)
              .withName("Green - Elevator Zeroed"));
      RobotModeTriggers.disabled()
          .and(hasBeenZeroed.negate())
          .onTrue(
              s.elevatorLEDSubsystem
                  .blink(120, 0, 0, "Red - Elevator Not Zeroed")
                  .ignoringDisable(true));
      subZeroPosition.whileTrue(
          Commands.waitSeconds(0.5)
              .andThen(
                  s.elevatorLEDSubsystem
                      .blink(120, 0, 0, "Red - Elevator Position Error")
                      .ignoringDisable(true)
                      .withName("Red - Elevator Position Error")));
    }
    RobotModeTriggers.autonomous()
        .whileTrue(s.elevatorLEDSubsystem.animate(LEDPattern.rainbow(255, 255), "Auto Rainbow"));
    Timer teleopTimer = new Timer();
    // when in teleop for less than 5 seconds after autononomous ends, restart the timer
    RobotModeTriggers.autonomous()
        .debounce(5, DebounceType.kFalling)
        .and(RobotModeTriggers.teleop())
        .onTrue(Commands.runOnce(() -> teleopTimer.restart()));
    RobotModeTriggers.teleop()
        .onFalse(Commands.runOnce(() -> teleopTimer.stop()).ignoringDisable(true));
    Shuffleboard.getTab("Controls").addDouble("Teleop time", () -> teleopTimer.get());
    new Trigger(() -> teleopTimer.hasElapsed(135 - 30))
        .onTrue(
            Commands.sequence(
                s.elevatorLEDSubsystem
                    .colorSet(255, 0, 0, "red half blink - 30 sec remaining")
                    .withTimeout(0.5),
                s.elevatorLEDSubsystem
                    .colorSet(255, 255, 0, "Yellow half blink - 30 sec remaining")
                    .withTimeout(0.5)));
    new Trigger(() -> teleopTimer.hasElapsed(135 - 15))
        .onTrue(
            Commands.sequence(
                s.elevatorLEDSubsystem
                    .colorSet(255, 0, 0, "Red half blink - 15 sec remaining")
                    .withTimeout(0.5),
                s.elevatorLEDSubsystem
                    .colorSet(255, 255, 0, "Yellow half blink - 15 sec remaining")
                    .withTimeout(0.5)));
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
    driverController
        .rightTrigger()
        .and(() -> scoringMode == ScoringMode.CORAL)
        .and(() -> branchHeight != BranchHeight.CORAL_LEVEL_ONE)
        .whileTrue(AutoAlign.autoAlignRight(s.drivebaseSubsystem, this));
    driverController
        .leftTrigger()
        .and(() -> scoringMode == ScoringMode.CORAL)
        .and(() -> branchHeight != BranchHeight.CORAL_LEVEL_ONE)
        .whileTrue(AutoAlign.autoAlignLeft(s.drivebaseSubsystem, this));
  }

  private void configureGroundSpinnyBindings() {
    if (s.groundSpinny == null) {
      return;
    }
    s.groundSpinny.setDefaultCommand(s.groundSpinny.holdFunnelIntakePower());
  }

  private void configureGroundArmBindings() {
    if (s.groundArm == null) {
      return;
    }
    s.groundArm.setDefaultCommand(
        s.groundArm
            .moveToPosition(GroundArm.STOWED_POSITION)
            .andThen(Commands.idle())
            .withName("Ground stowed position wait"));
    operatorController
        .rightBumper()
        .whileTrue(
            s.groundArm
                .moveToPosition(GroundArm.GROUND_POSITION)
                .andThen(Commands.idle())
                .withName("ground up position"));
  }

  private Command selectScoringHeight(BranchHeight b, AlgaeIntakeHeight a) {
    return Commands.runOnce(
            () -> {
              branchHeight = b;
              algaeIntakeHeight = a;
              if (intakeMode == ScoringMode.ALGAE
                  && (sensors.armSensor == null || !sensors.armSensor.booleanInClaw())) {
                CommandScheduler.getInstance().schedule(getAlgaeIntakeCommand());
              }
            })
        .alongWith(heightSelectRumble());
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

  private void configureSoloControllerBindings() {
    // Barge + Auto align left + Select scoring mode Algae
    soloController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Command scoreCommand;
                  switch (soloScoringMode) {
                    case CORAL_IN_CLAW -> {
                      scoreCommand = getSoloCoralBranchHeightCommand();
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
                                      s.drivebaseSubsystem, superStructure)
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
        .whileTrue(AutoAlign.autoAlignLeft(s.drivebaseSubsystem, this));
    // Processor + Auto align right + Select scoring mode Coral
    soloController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Command scoreCommand =
                          switch (soloScoringMode) {
                            case CORAL_IN_CLAW -> getSoloCoralBranchHeightCommand();
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
        .whileTrue(AutoAlign.autoAlignRight(s.drivebaseSubsystem, this));
    // Ground Intake
    soloController
        .leftBumper()
        .onTrue(
            superStructure
                .quickGroundIntake(soloController.povUp())
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
  }
}
