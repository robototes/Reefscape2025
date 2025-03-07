package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ArmSensor;
import java.util.function.BooleanSupplier;

public class SuperStructure {
  private final ElevatorSubsystem elevator;
  private final ArmPivot armPivot;
  private final SpinnyClaw spinnyClaw;
  private final ArmSensor armSensor;

  public SuperStructure(
      ElevatorSubsystem elevator, ArmPivot armPivot, SpinnyClaw spinnyClaw, ArmSensor armSensor) {
    this.elevator = elevator;
    this.armPivot = armPivot;
    this.spinnyClaw = spinnyClaw;
    this.armSensor = armSensor;
  }

  public Command coralLevelFour(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                Commands.print("Pre position"),
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                spinnyClaw.stop()),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_PRE_L4),
            Commands.waitUntil(score),
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L4)),
            spinnyClaw.coralHoldExtakePower().withTimeout(0.2),
            preIntake())
        .withName("Coral Level 4");
  }

  public Command coralLevelThree(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_POS),
        spinnyClaw.coralHoldExtakePower().withTimeout(0.15),
        preIntake().withName("Coral Level 3"));
  }

  public Command coralLevelTwo(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                spinnyClaw.stop()),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L2_L3),
            Commands.waitUntil(score),
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_POS),
            spinnyClaw.coralHoldExtakePower().withTimeout(0.15),
            preIntake())
        .withName("Coral Level 2");
  }

  public Command coralLevelOne(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_ONE_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L1),
                spinnyClaw.stop()),
            Commands.waitUntil(score),
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_POS),
            spinnyClaw.coralHoldExtakePower().withTimeout(0.15),
            preIntake())
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
                    spinnyClaw.intakePower(),
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
            // add a wait command here
            algaeStow())
        .withName("Algae L3-L4 Intake");
  }

  public Command algaeLevelTwoThreeIntake() { // theoretically
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
            // add a wait command here
            algaeStow())
        .withName("Algae L2-L3 Intake");
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

  public Command algaeLevelTwoThreeFling(BooleanSupplier finish) { // theoretically
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeFlingPower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_FLING),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE_FLING)),
            Commands.waitUntil(finish),
            algaeStow())
        .withName("Algae L2-L3 Fling");
  }

  public Command algaeLevelThreeFourPop() {
    return Commands.sequence(
            Commands.parallel(
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE_PREPOS),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)),
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
            algaeStow())
        .withName("Algae L3-L4 Pop");
  }

  public Command algaeLevelTwoThreePop() { // theoretically
    return Commands.sequence(
            Commands.parallel(
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE_PREPOS),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
            algaeStow())
        .withName("Algae L2-L3 Pop");
  }

  public Command algaeStow() { // Big North + Spider collab on this one
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_STOWED),
            armPivot.moveToPosition(ArmPivot.ALGAE_STOWED),
            spinnyClaw.algaeIntakePower())
        .withName("Algae Stow");
  }

  public Command algaeProcessorScore() { // Big North + Spider collab on this one
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE),
                armPivot.moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE),
                spinnyClaw.algaeIntakePower()),
            spinnyClaw.algaeExtakePower())
        .withName("Algae Processor Score");
  }
}
