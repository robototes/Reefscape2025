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
            elevator.setLevel(ElevatorSubsystem.LEVEL_FOUR_PRE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.PRESET_PRE_L4),
        Commands.waitUntil(score),
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.LEVEL_FOUR_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_L4)),
        spinnyClaw.holdExtakePower().withTimeout(0.2),
        preIntake());
  }

  public Command levelThree(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.LEVEL_THREE_PRE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.LEVEL_THREE_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        preIntake());
  }

  public Command levelTwo(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_PRE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        preIntake());
  }

  public Command levelOne(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.LEVEL_ONE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_L1),
            spinnyClaw.stop()),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        preIntake());
  }

  public Command stow() {
    return Commands.parallel(
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_STOWED),
        spinnyClaw.stop());
  }

  public Command preIntake() {
    return Commands.sequence(
        stow(),
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.PRE_INTAKE),
            armPivot.moveToPosition(ArmPivot.PRESET_DOWN),
            spinnyClaw.intakePower()));
  }

  public Command intake() {
    return Commands.sequence(
            elevator.setLevel(ElevatorSubsystem.INTAKE),
            Commands.waitSeconds(0.1),
            spinnyClaw.stop(),
            elevator.setLevel(ElevatorSubsystem.PRE_INTAKE),
            stow())
        .withName("Intake");
  }
}
