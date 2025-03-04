package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private Command animate(Animation animation) {
    if (elevatorLight == null) {
      return Commands.none();
    }
    return elevatorLight.animate(animation);
  }

  private Command colorSet(Color color, String name) {
    if (elevatorLight == null) {
      return Commands.none();
    }
    return elevatorLight.animate(LEDPattern.solid(color), name);
  }

  public Command levelFour(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            colorSet(new Color(0, 255, 0), null),
            elevator.setLevel(ElevatorSubsystem.LEVEL_FOUR_PRE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.PRESET_PRE_L4),
        Commands.waitUntil(score),
        Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.LEVEL_FOUR_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_L4)),
        spinnyClaw.holdExtakePower().withTimeout(0.2),
        stow());
  }

  public Command levelThree(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            colorSet(0, 255, 0),
            elevator.setLevel(ElevatorSubsystem.LEVEL_THREE_PRE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.LEVEL_THREE_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        stow());
  }

  private Command colorSet(int i, int j, int k) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'colorSet'");
  }

  public Command levelTwo(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            colorSet(0, 255, 0),
            elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_PRE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_UP),
            spinnyClaw.stop()),
        armPivot.moveToPosition(ArmPivot.PRESET_L2_L3),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        stow());
  }

  public Command levelOne(BooleanSupplier score) {
    return Commands.sequence(
        Commands.parallel(
            colorSet(0, 255, 0),
            elevator.setLevel(ElevatorSubsystem.LEVEL_ONE_POS),
            armPivot.moveToPosition(ArmPivot.PRESET_L1),
            spinnyClaw.stop()),
        Commands.waitUntil(score),
        elevator.setLevel(ElevatorSubsystem.LEVEL_TWO_POS),
        spinnyClaw.holdExtakePower().withTimeout(0.15),
        stow());
  }

  public Command stow() {
    return Commands.parallel(
        colorSet(255, 255, 255),
        elevator.setLevel(ElevatorSubsystem.STOWED),
        armPivot.moveToPosition(ArmPivot.PRESET_STOWED),
        spinnyClaw.stop());
  }

  public Command intake() {
    return Commands.sequence(
            Commands.parallel(
                colorSet(255, 92, 0),
                elevator.setLevel(ElevatorSubsystem.PRE_INTAKE),
                armPivot.moveToPosition(ArmPivot.PRESET_DOWN),
                spinnyClaw.intakePower()),
            // Commands.waitUntil(armSensor.inTrough()),
            colorSet(255, 255, 0),
            elevator.setLevel(ElevatorSubsystem.INTAKE),
            Commands.waitSeconds(0.1),
            spinnyClaw.stop(),
            elevator.setLevel(ElevatorSubsystem.PRE_INTAKE),
            stow())
        .withName("Intake");
  }
}
