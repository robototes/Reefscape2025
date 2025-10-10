package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ArmSensor;
import frc.robot.sensors.BranchSensors;
import frc.robot.sensors.ElevatorLight;
import frc.robot.sensors.IntakeSensor;
import frc.robot.util.BranchHeight;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SuperStructure {
  private final ElevatorSubsystem elevator;
  private final ArmPivot armPivot;
  private final SpinnyClaw spinnyClaw;
  private final GroundArm groundArm;
  private final GroundSpinny groundSpinny;
  private final ElevatorLight elevatorLight;
  private final ArmSensor armSensor;
  private final BranchSensors branchSensors;
  private final IntakeSensor intakeSensor;
  private Supplier<BranchHeight> branchHeight = () -> BranchHeight.CORAL_LEVEL_TWO;

  public SuperStructure(
      ElevatorSubsystem elevator,
      ArmPivot armPivot,
      SpinnyClaw spinnyClaw,
      GroundArm groundArm,
      GroundSpinny groundSpinny,
      ElevatorLight elevatorLight,
      ArmSensor armSensor,
      BranchSensors branchSensors,
      IntakeSensor intakeSensor) {
    this.elevator = elevator;
    this.armPivot = armPivot;
    this.spinnyClaw = spinnyClaw;
    this.groundArm = groundArm;
    this.groundSpinny = groundSpinny;
    this.elevatorLight = elevatorLight;
    this.armSensor = armSensor;
    this.branchSensors = branchSensors;
    this.intakeSensor = intakeSensor;
  }

  public void setBranchHeightSupplier(Supplier<BranchHeight> branchHeight) {
    this.branchHeight = branchHeight;
  }

  private Command colorSet(int r, int g, int b, String name) {
    if (elevatorLight == null) {
      return Commands.none();
    }
    return elevatorLight.colorSet(r, g, b, name);
  }

  private Command repeatPrescoreScoreSwing(Command command, BooleanSupplier score) {
    // repeats scoring sequence if the coral is still in the claw
    if (armSensor == null) {
      return Commands.sequence(
          command, Commands.waitUntil(() -> !score.getAsBoolean()), Commands.waitUntil(score));
    } else {
      return command.repeatedly().onlyWhile(armSensor.inClaw());
    }
  }

  public Command coralLevelFour(BooleanSupplier score) {
    if (branchSensors != null) { // checks if sensor enabled then use for faster scoring
      score = branchSensors.withinScoreRange().or(score);
    }
    return Commands.sequence(
            Commands.parallel(
                    Commands.print("Pre position"),
                    elevator
                        .setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS)
                        .deadlineFor( // keeps spinny claw engaged until coral has been scored
                            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP).until(score)),
                    spinnyClaw.stop())
                .withTimeout(0.7),
            repeatPrescoreScoreSwing(
                Commands.sequence(
                    Commands.parallel(
                            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS),
                            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_PRE_L4))
                        .withDeadline(
                            Commands.waitUntil(
                                score)), // waits until driver presses the score button or until
                    // auto scoring happens
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_DOWN)
                        .withTimeout(1.5)
                        .until(armPivot.atAngle(ArmPivot.CORAL_POST_SCORE))),
                score),
            Commands.print("Pre preIntake()"),
            coralPreIntake()
                .unless(
                    RobotModeTriggers
                        .autonomous()), // doesn't go to preintake if auto, should add for otehr
            // commands
            Commands.print("Post preIntake()"))
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L4").asProxy())
        .withName("Coral Level 4");
  }

  public Command coralLevelThree(BooleanSupplier score) { // same as L4
    return Commands.sequence(
            Commands.parallel(
                    elevator
                        .setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS)
                        .deadlineFor(
                            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP).until(score)),
                    spinnyClaw.stop())
                .withTimeout(0.5),
            repeatPrescoreScoreSwing(
                Commands.repeatingSequence(
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_L3)
                        .withDeadline(Commands.waitUntil(score)),
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_DOWN)
                        .withTimeout(1.5)
                        .until(armPivot.atAngle(ArmPivot.CORAL_POST_SCORE))),
                score),
            coralPreIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L3").asProxy())
        .withName("Coral Level 3");
  }

  public Command coralLevelTwo(BooleanSupplier score) { // same as L4 and L3
    return Commands.sequence(
            Commands.parallel(
                    elevator
                        .setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS)
                        .deadlineFor(
                            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP).until(score)),
                    spinnyClaw.stop())
                .withTimeout(0.5),
            repeatPrescoreScoreSwing(
                Commands.sequence(
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_L2)
                        .withDeadline(Commands.waitUntil(score)),
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_DOWN)
                        .withTimeout(1.5)
                        .until(armPivot.atAngle(ArmPivot.CORAL_POST_SCORE))),
                score),
            coralPreIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L2").asProxy())
        .withName("Coral Level 2");
  }

  public Command coralLevelOne(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                    elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_ONE_POS),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L1),
                    spinnyClaw.stop()) // holds coral without wearing flywheels
                .withTimeout(0.5)
                .withDeadline(Commands.waitUntil(score)),
            spinnyClaw.coralLevelOneHoldExtakePower().withTimeout(0.25), // spits out coral
            Commands.waitSeconds(1), // Wait to clear the reef
            coralPreIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L1").asProxy())
        .withName("Coral Level 1");
  }

  // quickGroundIntake() is used instead since it's faster and still reliable.
  // (This one moves the coral intake the normal intake position, then does the normal intake.
  // quickGroundIntake() instead hands the coral directly to the claw.)
  public Command groundIntake(BooleanSupplier retract) {
    if (groundSpinny == null || groundArm == null || intakeSensor == null) {
      return Commands.none().withName("ground intake disabled");
    } else {
      BooleanSupplier clawFull = armSensor != null ? armSensor.inClaw() : () -> false;
      return Commands.sequence(
              Commands.parallel(
                      elevator.setLevel(ElevatorSubsystem.CORAL_GROUND_INTAKE_POS),
                      armPivot.moveToPosition(ArmPivot.CORAL_PRESET_GROUND_INTAKE),
                      spinnyClaw.stop(), // just as a backup in case things are silly
                      groundSpinny.setGroundIntakePower())
                  .until(
                      elevator.above(
                          ElevatorSubsystem
                              .MIN_EMPTY_GROUND_INTAKE)), // waits until elevator is out of the way
              // to lift ground intake
              groundArm
                  .moveToPosition(GroundArm.GROUND_POSITION)
                  .andThen(groundArm.setVoltage(GroundArm.GROUND_HOLD_VOLTAGE))
                  .withDeadline(
                      Commands.waitUntil(
                          intakeSensor
                              .inIntake()
                              .or(retract))), // ends ground pickup to stow coral for scoring
              groundArm.moveToPosition(GroundArm.STOWED_POSITION),
              groundSpinny.setFunnelIntakePower(),
              coralPreIntake())
          .unless(clawFull)
          .withName("Ground Intake");
    }
  }

  // This is the actual version in use. It moves the coral directly into the claw.
  public Command quickGroundIntake(BooleanSupplier retract) { // thanks joseph
    if (groundSpinny == null || groundArm == null || intakeSensor == null) {
      // If anything's disabled, completely give up. (Theoretically we might be able to do some
      // stuff, but it wasn't worth implementing in the time we had.)
      return Commands.none().withName("quick ground intake disabled");
    } else {
      // BooleanSupplier for whether the claw is full. This is a separate variable so that it gets a
      // name (so it's easier to read).
      BooleanSupplier clawFull = armSensor != null ? armSensor.inClaw() : () -> false;
      // Make the big sequence.
      return Commands.sequence(
              // Initial setup- Move elevator high enough for ground arm to be clear, start moving
              // arm pivot, stop the spinny claw, and start spinning the ground intake
              Commands.parallel(
                      elevator.setLevel(ElevatorSubsystem.MIN_EMPTY_GROUND_INTAKE),
                      armPivot.moveToPosition(ArmPivot.CORAL_QUICK_INTAKE),
                      spinnyClaw.stop(), // just as a backup in case things are silly
                      groundSpinny.setGroundIntakePower())
                  // Move on even if arm isn't in position yet as long as elevator is high enough
                  .until(elevator.above(ElevatorSubsystem.MIN_EMPTY_GROUND_INTAKE)),
              // Core intake sequence
              Commands.sequence(
                      // Deploy the ground arm (and wait until it reaches the position).
                      groundArm.moveToPosition(GroundArm.GROUND_POSITION),
                      // After it's deployed, apply a constant voltage to press it into the bumper
                      // and continue.
                      groundArm.setVoltage(GroundArm.GROUND_HOLD_VOLTAGE),
                      // Grabbing segment
                      Commands.parallel(
                          // These three are the initial setup: Move elevator down to the handoff
                          // height, make sure armPivot finishes moving to the right height, and
                          // spin claw
                          elevator.setLevel(ElevatorSubsystem.CORAL_QUICK_INTAKE),
                          armPivot.moveToPosition(ArmPivot.CORAL_QUICK_INTAKE),
                          spinnyClaw.coralIntakePower(),
                          // Run the intake with occassional jitters. (This is stopped when the core
                          // intake sequence is stopped.)
                          Commands.sequence(
                                  // Wait until the jitter condition (stalling for at least 0.25
                                  // seconds and we don't have a coral)
                                  Commands.waitUntil(
                                      groundSpinny
                                          .stalling()
                                          .debounce(0.25)
                                          .and(intakeSensor.inIntake().negate())),
                                  // Then, spin out at the jitter speed for 0.1 seconds.
                                  groundSpinny.holdGroundIntakeJitterSpeed().withTimeout(0.1))
                              // Whenever we move on, make sure to set the ground spinny back to the
                              // ground intake speed. The finallyDo() ensures this happens no matter
                              // when we move on.
                              .finallyDo(() -> groundSpinny.imperativeSetGroundIntakePower())
                              // Repeat the wait and jitter until we move on.
                              .repeatedly()))
                  // Move on from the core intake sequence as soon as we either have something in
                  // ground intake or we tell it to retract.
                  .withDeadline(Commands.waitUntil(intakeSensor.inIntake().or(retract))),
              // Retract the groundArm all the way to stowed (so that we aren't slowing down to stop
              // in the middle), but move on to the next command when we're at the handoff point.
              groundArm
                  .moveToPosition(GroundArm.STOWED_POSITION)
                  .until(groundArm.atPosition(GroundArm.QUICK_INTAKE_POSITION)),
              // Spin groundSpinny out, but skip if we lost the coral.
              groundSpinny.setQuickHandoffExtakeSpeed().onlyIf(armSensor.inClaw()),
              // Go back to stow, but skip if we lost the coral.
              coralStow().onlyIf(armSensor.inClaw()),
              // If we lost the coral, reset for a quick retry.
              Commands.parallel(
                      elevator.setLevel(ElevatorSubsystem.MIN_EMPTY_GROUND_INTAKE),
                      armPivot.moveToPosition(ArmPivot.CORAL_QUICK_INTAKE),
                      spinnyClaw.stop())
                  .onlyIf(armSensor.inClaw().negate()))
          // Don't run the entire sequence at all if the claw is full.
          .unless(clawFull)
          .withName("Ground Intake");
    }
  }

  public Command coralStow() {
    return Commands.defer(
            () -> {
              BranchHeight currentBranchHeight = branchHeight.get();
              double elevatorHeight =
                  switch (currentBranchHeight) { // used to stow closer to the scoring height
                    case CORAL_LEVEL_FOUR -> ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS;
                    case CORAL_LEVEL_THREE -> ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS;
                    case CORAL_LEVEL_TWO -> ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS;
                    case CORAL_LEVEL_ONE -> ElevatorSubsystem.CORAL_LEVEL_ONE_POS;
                  };
              if (elevatorHeight > ElevatorSubsystem.CORAL_PRE_INTAKE) {
                return Commands.parallel(
                    elevator.setLevel(elevatorHeight),
                    Commands.sequence( // waits to raise the arm until elevator is above preintake
                        Commands.waitUntil(elevator.above(ElevatorSubsystem.CORAL_PRE_INTAKE)),
                        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP)),
                    spinnyClaw.stop());
              } else {
                return Commands.parallel(
                    Commands.sequence(
                        elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
                        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                        elevator.setLevel(elevatorHeight)),
                    spinnyClaw.stop());
              }
            },
            Set.of(elevator, armPivot, spinnyClaw))
        .withName("Coral Stow");
  }

  public Command autoCoralStow() {
    return Commands.defer(
            () ->
                Commands.parallel(
                    elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS),
                    Commands.sequence(
                        Commands.waitUntil(elevator.above(ElevatorSubsystem.CORAL_PRE_INTAKE)),
                        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP)),
                    spinnyClaw.stop()),
            Set.of(elevator, armPivot, spinnyClaw))
        .withName("Coral Stow");
  }

  public Command coralPreIntake() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN),
            spinnyClaw.coralRejectPower()) // keeps coral out while waiting to intake
        .andThen(Commands.print("end of preIntake()"))
        .withName("PreIntake");
  }

  public Trigger
      inCoralPreIntakePosition() { // creates a trigger for coral intake that waits until robot is
    // in preintake position
    return new Trigger(
        () ->
            elevator.atPosition(ElevatorSubsystem.CORAL_PRE_INTAKE)
                && armPivot.atPosition(ArmPivot.CORAL_PRESET_DOWN));
  }

  public Command coralIntake() { // yummy coral
    return Commands.sequence(
            Commands.sequence(
                Commands.parallel(
                    spinnyClaw.coralIntakePower(),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN)),
                elevator.setLevel(ElevatorSubsystem.CORAL_INTAKE_POS)),
            coralStow())
        .withName("Coral Intake");
  }

  public Command autoCoralIntake() { // yummy coral
    return Commands.sequence(
            Commands.sequence(
                Commands.parallel(
                    spinnyClaw.coralIntakePower(),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN)),
                elevator.setLevel(ElevatorSubsystem.CORAL_INTAKE_POS)),
            autoCoralStow())
        .withName("Coral Intake");
  }

  public Command algaeLevelThreeFourIntake() { // removes algae from upper reef
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)))
        .withName("Algae L3-L4 Intake");
  }

  public Command algaeLevelTwoThreeIntake() { // removes algae from lower reef
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)))
        .withName("Algae L2-L3 Intake");
  }

  public Command
      algaeGroundIntake() { // picks up algae on the ground (mostly only works against a wall)
    return Commands.parallel(
            spinnyClaw.algaeIntakePower(),
            Commands.sequence(
                armPivot.moveToPosition(ArmPivot.ALGAE_GROUND_INTAKE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_GROUND_INTAKE)))
        .withName("Algae Ground Intake");
  }

  public Command algaeStow() { // holds algae
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_STOWED),
            armPivot.moveToPosition(ArmPivot.ALGAE_STOWED),
            spinnyClaw.algaeGripIntakePower())
        .deadlineFor(colorSet(255, 255, 255, "White - Stowed").asProxy())
        .withName("Algae Stow");
  }

  public Command algaeProcessorScore(BooleanSupplier score) { // scores algae in processor
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeGripIntakePower(),
                Commands.sequence(
                    armPivot.moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE),
                    elevator.setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE))),
            Commands.waitUntil(score),
            spinnyClaw.algaeExtakeProcessorPower())
        .withName("Algae Processor Score");
  }

  public Command algaeNetPrescore() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP))
        .withName("Algae Net Prescore");
  }

  public Command algaeNetScore() {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.ALGAE_NET_SCORE),
                Commands.sequence(
                    spinnyClaw.algaeIntakePower(),
                    Commands.waitSeconds(0.25),
                    Commands.parallel(
                        armPivot.moveToPosition(ArmPivot.ALGAE_NET_SCORE),
                        Commands.sequence(
                            Commands.waitSeconds(0.05), spinnyClaw.algaeExtakePower())))),
            Commands.waitSeconds(0.2),
            armPivot.moveToPosition(
                ArmPivot.CORAL_PRESET_UP), // added to prevent hitting the barge after scoring net
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE))
        .withName("Algae Net Score");
  }

  /*public Command algaeLevelThreeFourFling(BooleanSupplier finish) {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeFlingPower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_FLING),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR_FLING)),
            Commands.waitUntil(finish),
            algaeStow())
        .withName("Algae L3-L4 Fling");
  }

  public Command algaeLevelTwoThreeFling(BooleanSupplier finish) {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeFlingPower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_FLING),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE_FLING)),
            Commands.waitUntil(finish),
            algaeStow())
        .withName("Algae L2-L3 Fling");
        }*/
}
