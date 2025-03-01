package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.BonkTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.util.AlgaeIntakeHeight;
import frc.robot.util.BranchHeight;
import frc.robot.util.RobotType;
import frc.robot.util.ScoringMode;
import java.util.Map;

public class Controls {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private static final int ARM_PIVOT_SPINNY_CLAW_CONTROLLER_PORT = 2;
  private static final int ELEVATOR_CONTROLLER_PORT = 3;
  private static final int CLIMB_TEST_CONTROLLER_PORT = 4;

  private final CommandXboxController driverController;

  private final CommandXboxController operatorController;

  private final CommandXboxController armPivotSpinnyClawController;

  private final CommandXboxController elevatorTestController;

  private final CommandXboxController climbTestController;
  private final Subsystems s;
  private final Sensors sensors;
  private final SuperStructure superStructure;

  private BranchHeight branchHeight = BranchHeight.LEVEL_FOUR;
  private ScoringMode scoringMode = ScoringMode.CORAL;
  private AlgaeIntakeHeight algaeIntakeHeight = AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR;

  // Swerve stuff
  private double MaxSpeed =
      RobotType.getCurrent() == RobotType.BONK
          ? BonkTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          : CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MAX_ACCELERATION = 50;
  private final double MAX_ROTATION_ACCELERATION = 50;
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

  public Controls(Subsystems s, Sensors sensors, SuperStructure superStructure) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    armPivotSpinnyClawController = new CommandXboxController(ARM_PIVOT_SPINNY_CLAW_CONTROLLER_PORT);
    elevatorTestController = new CommandXboxController(ELEVATOR_CONTROLLER_PORT);
    climbTestController = new CommandXboxController(CLIMB_TEST_CONTROLLER_PORT);
    this.s = s;
    this.sensors = sensors;
    this.superStructure = superStructure;
    configureDrivebaseBindings();
    configureSuperStructureBindings();
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
        s.drivebaseSubsystem
            .applyRequest(
                () -> {
                  ChassisSpeeds speed = s.drivebaseSubsystem.returnSpeeds();
                  ChassisSpeeds targetSpeeds =
                      new ChassisSpeeds(
                          -driverController.getLeftY()
                              * MaxSpeed, // Drive forward with negative Y (forward)
                          -driverController.getLeftX()
                              * MaxSpeed, // Drive left with negative X (left)
                          -driverController.getRightX()
                              * MaxAngularRate); // Drive counterclockwise with negative X (left)
                  ChassisSpeeds diff = targetSpeeds.minus(speed);
                  double dt = 0.02;
                  // Vx Vy and Omega are really accelerations and not velocities.
                  ChassisSpeeds acceleration = diff.div(dt);
                  // double translationAccelMagnitude =
                  //     Math.hypot(acceleration.vxMetersPerSecond, acceleration.vyMetersPerSecond);
                  // ChassisSpeeds translationLimit =
                  //     acceleration.times(
                  //         Math.min(1, Math.abs(MAX_ACCELERATION / translationAccelMagnitude)));
                  // ChassisSpeeds rotationLimit =
                  //     translationLimit.times(
                  //         Math.min(
                  //             1,
                  //             Math.abs(
                  //                 MAX_ROTATION_ACCELERATION
                  //                     / translationLimit.omegaRadiansPerSecond)));
                  // This *should* be rotationLimit, but the acceleration limiting causes the
                  // commanded speed to fall into the deadband. The proper fix is to do the deadband
                  // first, which relies on us doing the deadband ourselves, which is being
                  // difficult.
                  ChassisSpeeds newSpeeds = speed.plus(acceleration.times(dt));

                  return drive
                      .withVelocityX(newSpeeds.vxMetersPerSecond)
                      .withVelocityY(newSpeeds.vyMetersPerSecond)
                      .withRotationalRate(newSpeeds.omegaRadiansPerSecond);
                })
            .withName("Drive"));
    s.drivebaseSubsystem
        .applyRequest(() -> brake)
        .ignoringDisable(true)
        .withName("Brake")
        .schedule();

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
                .withName("Reset gyro"));
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
  }

  private void configureSuperStructureBindings() {
    if (superStructure == null) {
      return;
    }
    // operator start button used for climb - bound in climb bindings
    operatorController
        .y()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_FOUR).withName("level 4"))
        .onTrue(
            Commands.runOnce(() -> algaeIntakeHeight = AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR)
                .withName("algae level 3-4"));
    operatorController
        .x()
        .onTrue(
            Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_THREE).withName("level 3"));
    operatorController
        .b()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_TWO).withName("level 2"));
    operatorController
        .a()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_ONE).withName("level 1"))
        .onTrue(
            Commands.runOnce(() -> algaeIntakeHeight = AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("algae level 2-3"));
    ;
    driverController
        .povUp()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_FOUR).withName("level 4"))
        .onTrue(
            Commands.runOnce(() -> algaeIntakeHeight = AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR)
                .withName("algae level 3-4"));
    driverController
        .povLeft()
        .onTrue(
            Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_THREE).withName("level 3"));
    driverController
        .povRight()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_TWO).withName("level 2"));
    driverController
        .povDown()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_ONE).withName("level 1"))
        .onTrue(
            Commands.runOnce(() -> algaeIntakeHeight = AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE)
                .withName("algae level 2-3"));

    /*operatorController
    .a()
    .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.ALGAE_STOWED).withName("algae stow"));*/
    operatorController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(() -> scoringMode = ScoringMode.ALGAE).withName("Algae Scoring Mode"));
    operatorController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(() -> scoringMode = ScoringMode.CORAL).withName("Coral Scoring Mode"));
    operatorController
        .povLeft()
        .onTrue(
            Commands.select(
                    Map.of(
                        ScoringMode.CORAL,
                        superStructure.coralStow().asProxy(),
                        ScoringMode.ALGAE,
                        superStructure.algaeStow().asProxy()),
                    () -> scoringMode)
                .withName("Stow"));
    driverController
        .a()
        .onTrue(
            Commands.select(
                    Map.of(
                        ScoringMode.CORAL,
                        superStructure.coralIntake().asProxy(),
                        ScoringMode.ALGAE,
                        Commands.select(
                            Map.of(
                                AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR,
                                superStructure.algaeLevelThreeFourFling().asProxy(),
                                AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE,
                                superStructure.algaeLevelTwoThreeFling().asProxy()),
                            () -> algaeIntakeHeight)),
                    () -> scoringMode)
                .withName("Driver Intake"));
    driverController // just for testing
        .x()
        .onTrue(
            Commands.select(
                    Map.of(
                        AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR,
                        superStructure.algaeLevelThreeFourPop().asProxy(),
                        AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE,
                        superStructure.algaeLevelTwoThreePop().asProxy()),
                    () -> algaeIntakeHeight)
                .withName("Driver algae intake pop"));
    driverController // just for testing
        .y()
        .onTrue(
            Commands.select(
                    Map.of(
                        AlgaeIntakeHeight.ALGAE_LEVEL_THREE_FOUR,
                        superStructure.algaeLevelThreeFourIntake().asProxy(),
                        AlgaeIntakeHeight.ALGAE_LEVEL_TWO_THREE,
                        superStructure.algaeLevelTwoThreeIntake().asProxy()),
                    () -> algaeIntakeHeight)
                .withName("Driver algae intake"));
    if (sensors.armSensor != null) {
      sensors.armSensor.inTrough().onTrue(superStructure.coralIntake());
    }
    driverController
        .rightTrigger()
        .onTrue(s.elevatorSubsystem.runOnce(() -> {}).withName("elevator interruptor"))
        .onTrue(
            Commands.select(
                    Map.of(
                        ScoringMode.CORAL,
                        Commands.select(
                            Map.of(
                                BranchHeight.LEVEL_FOUR,
                                superStructure
                                    .coralLevelFour(driverController.rightBumper())
                                    .asProxy(),
                                BranchHeight.LEVEL_THREE,
                                superStructure
                                    .coralLevelThree(driverController.rightBumper())
                                    .asProxy(),
                                BranchHeight.LEVEL_TWO,
                                superStructure
                                    .coralLevelTwo(driverController.rightBumper())
                                    .asProxy(),
                                BranchHeight.LEVEL_ONE,
                                superStructure
                                    .coralLevelOne(driverController.rightBumper())
                                    .asProxy()),
                            () -> branchHeight),
                        ScoringMode.ALGAE,
                        superStructure.algaeProcessorScore().asProxy()),
                    () -> scoringMode)
                .withName("score"));
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
                .startMovingVoltage(() -> Volts.of(3 * -operatorController.getLeftY()))
                .withName("Elevator Manual Control"));
    s.elevatorSubsystem.setRumble(
        (rumble) -> {
          elevatorTestController.setRumble(RumbleType.kBothRumble, rumble);
          operatorController.setRumble(RumbleType.kBothRumble, rumble);
        });
    elevatorTestController
        .y()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_POS)
                .withName("Elevator L4"));
    elevatorTestController
        .x()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_POS)
                .withName("Elevator L3"));
    elevatorTestController
        .b()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_POS)
                .withName("Elevator L2"));
    elevatorTestController
        .a()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_LEVEL_ONE_POS)
                .withName("Elevator L1"));
    elevatorTestController
        .rightBumper()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.CORAL_INTAKE_POS)
                .withName("Elevator IntakePos"));
    elevatorTestController
        .povUp()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)
                .withName("Elevator Algae L3-L4"));
    elevatorTestController
        .povLeft()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)
                .withName("Elevator Algae L2-L3"));
    elevatorTestController
        .povRight()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_STOWED)
                .withName("Elevator Algae Stowed"));
    elevatorTestController
        .povDown()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE)
                .withName("Elevator Processor"));
    elevatorTestController.leftBumper().whileTrue(s.elevatorSubsystem.holdCoastMode());
    operatorController
        .back()
        .onTrue(
            Commands.parallel(
                    s.elevatorSubsystem.resetPosZero(),
                    Commands.startEnd(
                            () -> operatorController.setRumble(RumbleType.kBothRumble, 0.5),
                            () -> operatorController.setRumble(RumbleType.kBothRumble, 0))
                        .withTimeout(0.3))
                .ignoringDisable(true)
                .withName("Reset elevator zero"));
    operatorController.rightBumper().whileTrue(s.elevatorSubsystem.holdCoastMode());
  }

  private void configureArmPivotBindings() {
    if (s.armPivotSubsystem == null) {
      return;
    }

    // Arm Controls binding goes here
    operatorController // Untested :p
        .rightStick()
        .whileTrue(
            s.armPivotSubsystem
                .startMovingVoltage(() -> Volts.of(3 * operatorController.getRightY()))
                .withName("Arm Manual Control"));
    armPivotSpinnyClawController // Untested :p
        .rightStick()
        .whileTrue(
            s.armPivotSubsystem
                .startMovingVoltage(() -> Volts.of(3 * armPivotSpinnyClawController.getRightY()))
                .withName("Arm Manual Control"));
    armPivotSpinnyClawController
        .povRight()
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.CORAL_PRESET_L4).withName("Arm L4 Preset"));
    armPivotSpinnyClawController
        .povLeft()
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.CORAL_PRESET_L2_L3)
                .withName("Arm L2-L3 Preset"));
    armPivotSpinnyClawController
        .povUp()
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.CORAL_PRESET_UP).withName("Arm Preset Up"));
    armPivotSpinnyClawController
        .povDown()
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.CORAL_PRESET_DOWN)
                .withName("Arm Preset Down"));
    operatorController
        .povRight()
        .onTrue(s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_OUT).withName("Arm Preset Out"));
    armPivotSpinnyClawController
        .y()
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.ALGAE_REMOVE)
                .withName("Algae Preset Remove"));
    armPivotSpinnyClawController
        .b()
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE)
                .withName("Algae Preset Score"));
    armPivotSpinnyClawController
        .a()
        .onTrue(
            s.armPivotSubsystem
                .moveToPosition(ArmPivot.ALGAE_STOWED)
                .withName("Algae Preset Stowed"));
  }

  private void configureClimbPivotBindings() {
    if (s.climbPivotSubsystem == null) {
      return;
    }
    climbTestController.back().onTrue(s.climbPivotSubsystem.toggleClimb());
    climbTestController.start().onTrue(s.climbPivotSubsystem.zeroClimb());
    operatorController.start().onTrue(s.climbPivotSubsystem.toggleClimb());
  }

  private void configureSpinnyClawBindings() {
    if (s.spinnyClawSubsytem == null) {
      return;
    }
    // Claw controls bindings go here
    armPivotSpinnyClawController
        .leftBumper()
        .whileTrue(s.spinnyClawSubsytem.coralHoldExtakePower());
    armPivotSpinnyClawController
        .rightBumper()
        .whileTrue(s.spinnyClawSubsytem.coralHoldIntakePower());
    armPivotSpinnyClawController
        .leftTrigger()
        .whileTrue(s.spinnyClawSubsytem.algaeHoldExtakePower());
    armPivotSpinnyClawController
        .rightTrigger()
        .whileTrue(s.spinnyClawSubsytem.algaeHoldIntakePower());
  }

  private void configureElevatorLEDBindings() {
    if (s.elevatorLEDSubsystem == null) {
      return;
    }
    elevatorTestController
        .back()
        .onTrue(s.elevatorLEDSubsystem.animate(s.elevatorLEDSubsystem.larsonAnim));
    elevatorTestController
        .start()
        .onTrue(s.elevatorLEDSubsystem.animate(s.elevatorLEDSubsystem.rainbowAnim));
    if (s.elevatorSubsystem != null) {
      Trigger hasBeenZeroed = new Trigger(s.elevatorSubsystem::getHasBeenZeroed);
      Commands.waitSeconds(1)
          .andThen(
              s.elevatorLEDSubsystem.colorSet(50, 0, 0).withName("LED red").ignoringDisable(true))
          .schedule();
      hasBeenZeroed.onTrue(
          s.elevatorLEDSubsystem.colorSet(0, 50, 0).withName("LED green").ignoringDisable(true));
      hasBeenZeroed.onFalse(
          s.elevatorLEDSubsystem.colorSet(50, 0, 0).withName("LED red").ignoringDisable(false));
    }
  }
}
