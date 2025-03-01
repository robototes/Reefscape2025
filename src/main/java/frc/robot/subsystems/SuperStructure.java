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

  public Command coralLevelFour(BooleanSupplier score) {
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
        spinnyClaw.coralHoldExtakePower().withTimeout(0.2),
        coralStow());
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
        coralStow());
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
        coralStow());
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
        coralStow());
  }

  public Command coralStow() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.CORAL_STOWED),
        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_STOWED),
        spinnyClaw.stop());
  }

  public Command coralIntake() {
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
            coralStow())
        .withName("Intake");
  }

  public Command algaeLevelThreeFourIntake() {
    return Commands.sequence(
        Commands.parallel(
            spinnyClaw.algaeIntakePower(),
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)),
        // add a wait command here
        algaeStow());
  }

  public Command algaeLevelTwoThreeIntake() { // theoretically
    return Commands.sequence(
        Commands.parallel(
            spinnyClaw.algaeIntakePower(),
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
        // add a wait command here
        algaeStow());
  }

  public Command algaeLevelThreeFourFling() {
    return Commands.sequence(
        Commands.parallel(
            spinnyClaw.algaeFlingPower(),
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)),
        algaeStow());
  }

  public Command algaeLevelTwoThreeFling() { // theoretically
    return Commands.sequence(
        Commands.parallel(
            spinnyClaw.algaeFlingPower(),
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
        algaeStow());
  }

  public Command algaeLevelThreeFourPop() {
    return Commands.sequence(
        Commands.parallel(
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE_PREPOS),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)),
        armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
        algaeStow());
  }

  public Command algaeLevelTwoThreePop() { // theoretically
    return Commands.sequence(
        Commands.parallel(
            armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE_PREPOS),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
        armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
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
