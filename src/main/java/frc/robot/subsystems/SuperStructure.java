package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ArmSensor;
import frc.robot.sensors.ElevatorLight;
import java.util.function.BooleanSupplier;

public class SuperStructure {
  private final ElevatorSubsystem elevator;
  private final ArmPivot armPivot;
  private final SpinnyClaw spinnyClaw;
  private final ElevatorLight elevatorLight;
  private final ArmSensor armSensor;

  public SuperStructure(
      ElevatorSubsystem elevator,
      ArmPivot armPivot,
      SpinnyClaw spinnyClaw,
      ElevatorLight elevatorLight,
      ArmSensor armSensor) {
    this.elevator = elevator;
    this.armPivot = armPivot;
    this.spinnyClaw = spinnyClaw;
    this.elevatorLight = elevatorLight;
    this.armSensor = armSensor;
  }

  private Command colorSet(int r, int g, int b, String name) {
    if (elevatorLight == null) {
      return Commands.none();
    }
    return elevatorLight.colorSet(r, g, b, name);
  }

  public Command coralLevelFour(
      BooleanSupplier score) { // when we change L4, add repeating score sequence
    return Commands.sequence(
            Commands.parallel(
                    Commands.print("Pre position"),
                    elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                    spinnyClaw.stop())
                .withTimeout(2.0),
            armPivot
                .moveToPosition(ArmPivot.CORAL_PRESET_PRE_L4)
                .withTimeout(1.0)
                .withDeadline(Commands.waitUntil(score)),
            Commands.parallel(
                    elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_POS),
                    (armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L4)))
                .withTimeout(2.0),
            Commands.repeatingSequence(
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L4),
                    Commands.waitUntil(score),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN))
                .onlyWhile(armSensor.inClaw()),
            Commands.print("Pre preIntake()"),
            preIntake(),
            Commands.print("Post preIntake()"))
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L4").asProxy())
        .withName("Coral Level 4");
  }

  public Command coralLevelThree(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator
                    .setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS)
                    .deadlineFor(armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP)),
                spinnyClaw.stop()),
            Commands.repeatingSequence(
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_L3)
                        .withDeadline(Commands.waitUntil(score)),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN))
                .onlyWhile(armSensor.inClaw()),
            preIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L3").asProxy())
        .withName("Coral Level 3");
  }

  public Command coralLevelTwo(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator
                    .setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS)
                    .deadlineFor(armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP)),
                spinnyClaw.stop()),
            Commands.repeatingSequence(
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_L2)
                        .withDeadline(Commands.waitUntil(score)),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN))
                .onlyWhile(armSensor.inClaw()),
            preIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L2").asProxy())
        .withName("Coral Level 2");
  }

  public Command coralLevelOne(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_ONE_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L1),
                spinnyClaw.stop()),
            Commands.waitUntil(score),
            spinnyClaw.coralHoldExtakePower().withTimeout(0.25),
            preIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L1").asProxy())
        .withName("Coral Level 1");
  }

  public Command coralStow() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_STOWED),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_STOWED),
            spinnyClaw.stop())
        .withName("Coral Stow");
  }

  public Command preIntake() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN),
            spinnyClaw.stop())
        .andThen(Commands.print("end of preIntake()"))
        .withName("PreIntake");
  }

  public Trigger inPreIntakePosition() {
    return new Trigger(
        () ->
            elevator.atPosition(ElevatorSubsystem.CORAL_PRE_INTAKE)
                && armPivot.atPosition(ArmPivot.CORAL_PRESET_DOWN));
  }

  public Command coralIntake() {
    return Commands.sequence(
            Commands.sequence(
                    Commands.parallel(
                        spinnyClaw.coralIntakePower(),
                        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN)),
                    elevator.setLevel(ElevatorSubsystem.CORAL_INTAKE_POS),
                    Commands.idle())
                .until(armSensor.inClaw()),
            spinnyClaw.stop(),
            elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
            coralStow())
        .withName("Coral Intake");
  }

  public Command algaeLevelThreeFourIntake() {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)),
            Commands.waitUntil(armSensor.inClaw()), // if statement to check armsensor,
            algaeStow())
        .withName("Algae L3-L4 Intake");
  }

  public Command algaeLevelTwoThreeIntake() {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
            Commands.waitUntil(
                armSensor
                    .inClaw()), // add if statement to check armsensor, if not, then regular wait
            // for 1 sec
            algaeStow())
        .withName("Algae L2-L3 Intake");
  }

  public Command algaeGroundIntake() {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_GROUND_INTAKE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_GROUND_INTAKE)),
            Commands.waitUntil(
                armSensor
                    .inClaw()), // add if statement to check armsensor, if not, then regular wait
            // for 1 sec
            algaeStow())
        .withName("Algae Ground Intake");
  }

  public Command algaeLevelThreeFourFling(BooleanSupplier finish) {
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
  }

  public Command algaeStow() { // Big North + Spider collab on this one
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_STOWED),
            armPivot.moveToPosition(ArmPivot.ALGAE_STOWED),
            spinnyClaw.algaeGripIntakePower())
        .deadlineFor(colorSet(255, 255, 255, "White - Stowed").asProxy())
        .withName("Algae Stow");
  }

  public Command algaeProcessorScore(BooleanSupplier score) { // Big North + Spider collab on this one
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE),
                armPivot.moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE),
                spinnyClaw.algaeIntakePower()),
            Commands.waitUntil(score),
            spinnyClaw.algaeHoldExtakePower().withTimeout(0.25))
        .withName("Algae Processor Score");
  }

  public Command algaeNetScore(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.ALGAE_NET_SCORE),
                armPivot.moveToPosition(ArmPivot.ALGAE_NET_SCORE),
                spinnyClaw.algaeIntakePower()),
                Commands.waitUntil(score),
            spinnyClaw.algaeHoldExtakePower().withTimeout(0.25))
        .withName("Algae Net Score");
  }
}
