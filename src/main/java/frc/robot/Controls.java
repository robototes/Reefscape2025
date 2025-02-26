package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.BonkTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.util.BranchHeight;
import frc.robot.util.RobotType;
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

  private BranchHeight branchHeight = null;

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
    configureElevatorBindings();
    configureArmPivotBindings();
    configureClimbPivotBindings();
    configureSpinnyClawBindings();
    configureSuperStructureBindings();
    configureElevatorLEDBindings();
  }

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }

    double getLeftX = MathUtil.applyDeadband(driverController.getLeftX(),0.1);
    double getLeftY = MathUtil.applyDeadband(driverController.getLeftY(), 0.1);
    double getRightX = MathUtil.applyDeadband(driverController.getRightX(), 0.1);
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
                          -getLeftX
                              * MaxSpeed
                              * (driverController.start().getAsBoolean()
                                  ? 0.5
                                  : 1), // Drive forward with negative Y (forward)
                          -getLeftY
                              * MaxSpeed
                              * (driverController.start().getAsBoolean()
                                  ? 0.5
                                  : 1), // Drive left with negative X (left)
                          -getRightX
                              * MaxAngularRate
                              * (driverController.start().getAsBoolean()
                                  ? 0.5
                                  : 1)); // Drive counterclockwise with negative X (left)
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
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_FOUR).withName("level 4"));
    operatorController
        .x()
        .onTrue(
            Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_THREE).withName("level 3"));
    operatorController
        .b()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_TWO).withName("level 2"));
    operatorController
        .a()
        .onTrue(Commands.runOnce(() -> branchHeight = BranchHeight.LEVEL_ONE).withName("level 1"));
    operatorController.rightBumper().onTrue(superStructure.stow().withName("Stow"));
    driverController.a().onTrue(superStructure.intake());
    if (sensors.armSensor != null) {
      sensors.armSensor.inTrough().onTrue(superStructure.intake());
    }
    driverController
        .leftBumper()
        .onTrue(s.elevatorSubsystem.runOnce(() -> {}).withName("elevator interruptor"))
        .onTrue(
            Commands.select(
                    Map.of(
                        BranchHeight.LEVEL_FOUR,
                        superStructure.levelFour(driverController.rightBumper()),
                        BranchHeight.LEVEL_THREE,
                        superStructure.levelThree(driverController.rightBumper()),
                        BranchHeight.LEVEL_TWO,
                        superStructure.levelTwo(driverController.rightBumper()),
                        BranchHeight.LEVEL_ONE,
                        superStructure.levelOne(driverController.rightBumper())),
                    () -> branchHeight)
                .withName("go to target branch height"));
  }

  private void configureElevatorBindings() {
    if (s.elevatorSubsystem == null) {
      return;
    }
    RobotModeTriggers.disabled().onTrue(s.elevatorSubsystem.stop());
    // Controls binding goes here
    operatorController
        .leftTrigger(0.1)
        .whileTrue(
            s.elevatorSubsystem
                .goUpPower(
                    () -> MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1))
                .withName("Power up"));
    operatorController
        .rightTrigger(0.1)
        .whileTrue(
            s.elevatorSubsystem
                .goDownPower(
                    () -> MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1))
                .withName("Power down"));
    // operatorController
    //     .leftTrigger()
    //     .whileTrue(s.elevatorSubsystem.sysIdDynamic(Direction.kForward));
    // operatorController
    //     .leftBumper()
    //     .whileTrue(s.elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
    // operatorController
    //     .rightTrigger()
    //     .whileTrue(s.elevatorSubsystem.sysIdDynamic(Direction.kReverse));
    // operatorController
    //    .rightBumper()
    //    .whileTrue(s.elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
    s.elevatorSubsystem.setRumble(
        (rumble) -> {
          elevatorTestController.setRumble(RumbleType.kBothRumble, rumble);
          operatorController.setRumble(RumbleType.kBothRumble, rumble);
        });
    elevatorTestController
        .y()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.LEVEL_FOUR_POS).withName("Elevator L4"));
    elevatorTestController
        .x()
        .onTrue(
            s.elevatorSubsystem
                .setLevel(ElevatorSubsystem.LEVEL_THREE_POS)
                .withName("Elevator L3"));
    elevatorTestController
        .b()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.LEVEL_TWO_POS).withName("Elevator L2"));
    elevatorTestController
        .a()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.LEVEL_ONE_POS).withName("Elevator L1"));
    elevatorTestController
        .rightBumper()
        .onTrue(
            s.elevatorSubsystem.setLevel(ElevatorSubsystem.INTAKE).withName("Elevator IntakePos"));
    operatorController.povUp().whileTrue(s.elevatorSubsystem.goUp().withName("Elevator go up"));
    operatorController
        .povDown()
        .whileTrue(s.elevatorSubsystem.goDown().withName("Elevator go down"));
    operatorController
        .leftBumper()
        .onTrue(
            Commands.parallel(
                    s.elevatorSubsystem.resetPosZero(),
                    Commands.startEnd(
                            () -> operatorController.setRumble(RumbleType.kBothRumble, 0.5),
                            () -> operatorController.setRumble(RumbleType.kBothRumble, 0))
                        .withTimeout(0.3))
                .ignoringDisable(true)
                .withName("Reset elevator zero"));
  }

  private void configureArmPivotBindings() {
    if (s.armPivotSubsystem == null) {
      return;
    }

    // Arm Controls binding goes here
    armPivotSpinnyClawController
        .a()
        .whileTrue(s.armPivotSubsystem.SysIDDynamic(Direction.kForward));
    armPivotSpinnyClawController
        .b()
        .whileTrue(s.armPivotSubsystem.SysIDDynamic(Direction.kReverse));
    armPivotSpinnyClawController
        .x()
        .whileTrue(s.armPivotSubsystem.SysIDQuasistatic(Direction.kForward));
    armPivotSpinnyClawController
        .y()
        .whileTrue(s.armPivotSubsystem.SysIDQuasistatic(Direction.kReverse));
    armPivotSpinnyClawController
        .leftStick()
        .whileTrue(
            s.armPivotSubsystem
                .startMovingVoltage(() -> Volts.of(3 * armPivotSpinnyClawController.getLeftY()))
                .withName("ManuallyMoveArm"));
    armPivotSpinnyClawController
        .povRight()
        .onTrue(s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_L4).withName("SetArmPresetL4"));
    armPivotSpinnyClawController
        .povLeft()
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_L2_L3).withName("SetArmPresetL2_3"));
    armPivotSpinnyClawController
        .povUp()
        .onTrue(s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_UP).withName("SetArmPresetUp"));
    armPivotSpinnyClawController
        .povDown()
        .onTrue(
            s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_DOWN).withName("SetArmPresetDown"));
    operatorController
        .povRight()
        .onTrue(s.armPivotSubsystem.moveToPosition(ArmPivot.PRESET_OUT).withName("ArmPivotOut"));
  }

  private void configureClimbPivotBindings() {
    if (s.climbPivotSubsystem == null) {
      return;
    }
    // Idk if this is great code or horrible code
    climbTestController.back().onTrue(s.climbPivotSubsystem.toggleClimb());
    climbTestController.start().onTrue(s.climbPivotSubsystem.zeroClimb());
  }

  private void configureSpinnyClawBindings() {
    if (s.spinnyClawSubsytem == null) {
      return;
    }
    // Claw controls bindings go here
    armPivotSpinnyClawController.rightBumper().whileTrue(s.spinnyClawSubsytem.holdExtakePower());
    armPivotSpinnyClawController.leftBumper().whileTrue(s.spinnyClawSubsytem.holdIntakePower());
    driverController.leftTrigger().whileTrue(s.spinnyClawSubsytem.holdExtakePower());
    driverController.rightTrigger().whileTrue(s.spinnyClawSubsytem.holdIntakePower());
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
