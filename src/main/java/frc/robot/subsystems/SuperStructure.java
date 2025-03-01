package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command levelFour(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_PRE_L4),
        Commands.waitUntil(score),
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_POS),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L4)),
        spinnyClaw.holdExtakePower().withTimeout(0.2),
        stow());
  }

  public Command levelThree(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        stow());
  }

  public Command levelTwo(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        stow());
  }

  public Command levelOne(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_ONE_POS),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L1),
            spinnyClaw.stop()),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        stow());
  }

  public Command stow() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.CORAL_STOWED),
        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_STOWED),
        spinnyClaw.stop());
  }

  public Command intake() {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN),
                spinnyClaw.intakePower()),
            // Commands.waitUntil(armSensor.inTrough()),
            elevator.setLevel(ElevatorSubsystem.CORAL_INTAKE_POS),
            Commands.waitSeconds(0.1),
            spinnyClaw.stop(),
            elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
            stow())
        .withName("Intake");
  }

  public Command algaeLevelThreeFourIntake() {
    return Commands.sequence(
        spinnyClaw.algaeIntakePower(),
        armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
        elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR),
        algaeStow());
  }

  public Command algaeLevelTwoThreeIntake() { // theoretically
    return Commands.sequence(
        spinnyClaw.algaeIntakePower(),
        armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
        elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE),
        algaeStow());
  }

  public Command algaeLevelThreeFourFling() {
    return Commands.sequence(
        spinnyClaw.algaeFlingPower(),
        armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
        elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR),
        algaeStow());
  }

  public Command algaeLevelTwoThreeFling() { // theoretically
    return Commands.sequence(
        spinnyClaw.algaeFlingPower(),
        armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
        elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE),
        algaeStow());
  }

  public Command algaeStow() { // Big North + Spider collab on this one
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.ALGAE_STOWED),
        armPivot.moveToPosition(ArmPivot.ALGAE_STOWED),
        spinnyClaw.algaeIntakePower());
  }

  public Command algaeProcessorScore() { // Big North + Spider collab on this one
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE),
            armPivot.moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE),
            spinnyClaw.algaeIntakePower()),
        spinnyClaw.algaeExtakePower());
  }
}
