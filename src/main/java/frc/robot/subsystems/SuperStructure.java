package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

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
        elevator.setLevel(ElevatorSubsystem.LEVEL_FOUR_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L4),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelFourScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L4),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelFour(BooleanSupplier score) {
    return Commands.sequence(
        levelFourPrescore(),
        Commands.waitUntil(score),
        levelFourScore(),
        intake(score, score) // errors are gross and i'm lazy
        );
  }

  public Command levelThreePrescore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.LEVEL_THREE_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelThreeScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelThree(BooleanSupplier score) {
    return Commands.sequence(
        levelThreePrescore(),
        Commands.waitUntil(score),
        levelThreeScore(),
        intake(score, score) // again, running from errors
        );
  }

  public Command levelTwoPrescore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelTwoScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelTwo(BooleanSupplier score) {
    return Commands.sequence(
        levelTwoPrescore(),
        Commands.waitUntil(score),
        levelTwoScore(),
        intake(score, score) // ok now i just don't want annoying errors
        );
  }

  public Command levelOnePrescore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.LEVEL_ONE_POS),
        armPivot.moveToPosition(ArmPivot.PRESET_L1),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelOneScore() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_L1),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command levelOne(BooleanSupplier score, BooleanSupplier intake, BooleanSupplier holding) {
    return Commands.sequence(
        levelOnePrescore(),
        Commands.waitUntil(score),
        levelOneScore(),
        intake(
            intake,
            holding) // ok this is iffy and now im just making things up... unsure whether this
        // works
        );
  }

  public Command stow() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_DOWN),
        spinnyClaw.movingVoltage(() -> Volts.zero()));
  }

  public Command intake(
      BooleanSupplier intake,
      BooleanSupplier holding) { // not sure if boolean supplier will work to check using sensors
    return Commands.parallel(
        stow(),
        Commands.waitUntil(intake),
        elevator.setLevel(ElevatorSubsystem.INTAKE),
        armPivot.moveToPosition(ArmPivot.PRESET_DOWN),
        spinnyClaw.movingVoltage(() -> Volts.zero()), // should eventually spin to intake
        Commands.waitUntil(holding),
        stow());
  }
}
