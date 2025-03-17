package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ArmSensor;
import frc.robot.sensors.ElevatorLight;
import java.util.function.BooleanSupplier;
import frc.robot.util.AlgaeIntakeHeight;

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

  public Command coralLevelFour(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                    Commands.print("Pre position"),
                    elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                    spinnyClaw.stop())
                .withTimeout(2.0),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_PRE_L4).withTimeout(1.0),
            Commands.waitUntil(score),
            Commands.parallel(
                    elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_POS),
                    armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L4))
                .withTimeout(2.0),
            spinnyClaw.coralHoldExtakeL4Power().withTimeout(0.2),
            Commands.print("Pre preIntake()"),
            preIntake(),
            Commands.print("Post preIntake()"))
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L4").asProxy())
        .withName("Coral Level 4");
  }

  public Command coralLevelThree(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_THREE_PRE_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                spinnyClaw.stop()),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L3),
            Commands.waitUntil(score),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN),
            preIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L3").asProxy())
        .withName("Coral Level 3");
  }

  public Command coralLevelTwo(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_TWO_PRE_POS),
                armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
                spinnyClaw.stop()),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_L2),
            Commands.waitUntil(score),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN),
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
            spinnyClaw.coralHoldExtakePower().withTimeout(0.15),
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
                        spinnyClaw.intakePower(),
                        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN)),
                    elevator.setLevel(ElevatorSubsystem.CORAL_INTAKE_POS),
                    Commands.idle())
                .until(armSensor.inClaw()),
            spinnyClaw.stop(),
            elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
            coralStow())
        .withName("Coral Intake");
  }

  // ALGAE INTAKING
  public Command algaeGroundIntake() {
    return Commands.sequence(
            Commands.parallel(
                    spinnyClaw.algaeIntakePower(),
                    armPivot.moveToPosition(ArmPivot.ALGAE_GROUND_INTAKE),
                    elevator.setLevel(ElevatorSubsystem.ALGAE_GROUND_INTAKE))
                .until(armSensor.inClaw()),
            algaeStow())
        .withName("Algae Ground Intake");
  }

  public Command algaeLevelTwoThreeIntake() {
    return Commands.sequence(
            Commands.parallel(
                    spinnyClaw.algaeIntakePower(),
                    armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                    elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE))
                .until(armSensor.inClaw()),
            algaeStow())
        .withName("Algae L2-L3 Intake");
  }

  public Command algaeLevelThreeFourIntake() {
    return Commands.sequence(
            Commands.parallel(
                    spinnyClaw.algaeIntakePower(),
                    armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                    elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR))
                .until(armSensor.inClaw()),
            algaeStow())
        .withName("Algae L3-L4 Intake");
  }

  // public Command algaeLevelThreeFourFling(BooleanSupplier finish) {
  //   return Commands.sequence(
  //           Commands.parallel(
  //               spinnyClaw.algaeFlingPower(),
  //               armPivot.moveToPosition(ArmPivot.ALGAE_FLING),
  //               elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR_FLING)),
  //           Commands.waitUntil(finish),
  //           algaeStow())
  //       .withName("Algae L3-L4 Fling");
  // }

  // public Command algaeLevelTwoThreeFling(BooleanSupplier finish) { // theoretically
  //   return Commands.sequence(
  //           Commands.parallel(
  //               spinnyClaw.algaeFlingPower(),
  //               armPivot.moveToPosition(ArmPivot.ALGAE_FLING),
  //               elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE_FLING)),
  //           Commands.waitUntil(finish),
  //           algaeStow())
  //       .withName("Algae L2-L3 Fling");
  // }

  // public Command algaeLevelThreeFourPop() {
  //   return Commands.sequence(
  //           Commands.parallel(
  //               armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE_PREPOS),
  //               elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)),
  //           armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
  //           algaeStow())
  //       .withName("Algae L3-L4 Pop");
  // }

  // public Command algaeLevelTwoThreePop() { // theoretically
  //   return Commands.sequence(
  //           Commands.parallel(
  //               armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE_PREPOS),
  //               elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)),
  //           armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
  //           algaeStow())
  //       .withName("Algae L2-L3 Pop");
  // }

  //Hold Algae
  public Command algaeStow() { // Big North + Spider collab on this one
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_STOWED),
            armPivot.moveToPosition(ArmPivot.ALGAE_STOWED),
            spinnyClaw.algaeHoldIntakePower())
        .deadlineFor(colorSet(255, 255, 255, "White - Stowed").asProxy())
        .withName("Algae Stow");
  }

  //Processor
  public Command algaeProcessorScore(BooleanSupplier score, AlgaeIntakeHeight algaeIntakeHeight) { // Big North + Spider collab on this one
    return Commands.sequence(
      Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_PROCESSOR_SCORE),
            armPivot.moveToPosition(ArmPivot.ALGAE_PROCESSOR_SCORE)),
        Commands.waitUntil(score),
            spinnyClaw.algaeExtakePower(),
        Commands.waitSeconds(0.7),
        Commands.deferredProxy(
          () ->
              switch (algaeIntakeHeight) {
                  case ALGAE_LEVEL_THREE_FOUR -> algaeLevelThreeFourIntake();
                  case ALGAE_LEVEL_TWO_THREE -> algaeLevelTwoThreeIntake();
                  case ALGAE_LEVEL_GROUND -> algaeGroundIntake();
                })
      ).withName("Algae Processor Score");
  }

  //Net
  public Command algaeNetScore(BooleanSupplier score, AlgaeIntakeHeight algaeIntakeHeight) {
    return Commands.sequence(
      Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_NET_HEIGHT),
            armPivot.moveToPosition(ArmPivot.ALGAE_NET_ANGLE)),
        Commands.waitUntil(score),
            spinnyClaw.algaeExtakePower(),
        Commands.waitSeconds(0.7),
        Commands.deferredProxy(
          () ->
              switch (algaeIntakeHeight) {
                  case ALGAE_LEVEL_THREE_FOUR -> algaeLevelThreeFourIntake();
                  case ALGAE_LEVEL_TWO_THREE -> algaeLevelTwoThreeIntake();
                  case ALGAE_LEVEL_GROUND -> algaeGroundIntake();
                })
    ).withName("Algae Net Score");
  }
}
