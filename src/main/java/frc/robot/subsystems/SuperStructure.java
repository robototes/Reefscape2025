package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;

public class SuperStructure {
  private final ElevatorSubsystem elevator;
  private final ArmPivot armPivot;
  private final SpinnyClaw spinnyClaw;

  public SuperStructure(ElevatorSubsystem elevator, ArmPivot armPivot, SpinnyClaw spinnyClaw) {
    this.elevator = elevator;
    this.armPivot = armPivot;
    this.spinnyClaw = spinnyClaw;
  }

  public Command levelFourPrescore() {
    return Commands.parallel(
        // may need to make this a sequential if elevator/arm move at different times
        elevator.setLevel(ElevatorSubsystem.LEVEL_FOUR_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L4),
        spinnyClaw.stop());
  }

  public Command levelFourScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L4),
        spinnyClaw.extakePower());
  }

  public Command levelFour(BooleanSupplier score) {
    return Commands.sequence(
        levelFourPrescore(), Commands.waitUntil(score), levelFourScore(), stow());
  }

  public Command levelThreePrescore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.LEVEL_THREE_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.stop());
  }

  public Command levelThreeScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.extakePower());
  }

  public Command levelThree(BooleanSupplier score) {
    return Commands.sequence(
        levelThreePrescore(), Commands.waitUntil(score), levelThreeScore(), stow());
  }

  public Command levelTwoPrescore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.stop());
  }

  public Command levelTwoScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.extakePower());
  }

  public Command levelTwo(BooleanSupplier score) {
    return Commands.sequence(
        levelTwoPrescore(), Commands.waitUntil(score), levelTwoScore(), stow());
  }

  public Command levelOnePrescore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.LEVEL_ONE_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L1),
        spinnyClaw.stop());
  }

  public Command levelOneScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L1),
        spinnyClaw.extakePower());
  }

  public Command levelOne(BooleanSupplier score) {
    return Commands.sequence(
        levelOnePrescore(), Commands.waitUntil(score), levelOneScore(), stow());
  }

  public Command stow() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_DOWN),
        spinnyClaw.stop());
  }

  public Command intake(BooleanSupplier intake, BooleanSupplier holding) {
    // not sure if boolean supplier will work to check using sensors
    return Commands.sequence(
        stow(),
        Commands.waitUntil(intake),
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.INTAKE),
            armPivot.moveToPosition(ArmPivot.PRESET_DOWN),
            spinnyClaw.intakePower()),
        Commands.waitUntil(holding),
        stow());
  }
}
