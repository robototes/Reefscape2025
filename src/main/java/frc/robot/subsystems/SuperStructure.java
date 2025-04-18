package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.ArmSensor;
import frc.robot.sensors.BranchSensors;
import frc.robot.sensors.ElevatorLight;
import frc.robot.sensors.IntakeSensor;
import java.util.function.BooleanSupplier;

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

  private Command colorSet(int r, int g, int b, String name) {
    if (elevatorLight == null) {
      return Commands.none();
    }
    return elevatorLight.colorSet(r, g, b, name);
  }

  private Command repeatPrescoreScoreSwing(Command command, BooleanSupplier score) {
    if (armSensor == null) {
      return Commands.sequence(
          command, Commands.waitUntil(() -> !score.getAsBoolean()), Commands.waitUntil(score));
    } else {
      return command.repeatedly().onlyWhile(armSensor.inClaw());
    }
  }

  private Command untilClawFull(Command command) {
    if (armSensor == null) {
      return command;
    } else {
      return command.withDeadline(Commands.waitUntil(armSensor.inClaw()));
    }
  }

  public Command coralLevelFour(BooleanSupplier score) {
    if (branchSensors != null) {
      score = branchSensors.withinScoreRange().or(score);
    }
    return Commands.sequence(
            Commands.parallel(
                    Commands.print("Pre position"),
                    elevator
                        .setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS)
                        .deadlineFor(
                            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP).until(score)),
                    spinnyClaw.stop())
                .withTimeout(0.7),
            repeatPrescoreScoreSwing(
                Commands.sequence(
                    Commands.parallel(
                            elevator.setLevel(ElevatorSubsystem.CORAL_LEVEL_FOUR_PRE_POS),
                            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_PRE_L4))
                        .withDeadline(Commands.waitUntil(score)),
                    armPivot
                        .moveToPosition(ArmPivot.CORAL_PRESET_DOWN)
                        .withTimeout(1.5)
                        .until(armPivot.atAngle(ArmPivot.CORAL_POST_SCORE))),
                score),
            Commands.print("Pre preIntake()"),
            coralPreIntake(),
            Commands.print("Post preIntake()"))
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L4").asProxy())
        .withName("Coral Level 4");
  }

  public Command coralLevelThree(BooleanSupplier score) {
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

  public Command coralLevelTwo(BooleanSupplier score) {
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
                    spinnyClaw.stop())
                .withTimeout(0.5)
                .withDeadline(Commands.waitUntil(score)),
            spinnyClaw.coralHoldExtakePower().withTimeout(0.25),
            Commands.waitSeconds(1), // Wait to clear the reef
            coralPreIntake())
        .deadlineFor(colorSet(0, 255, 0, "Green - Aligned With L1").asProxy())
        .withName("Coral Level 1");
  }

  public Command groundIntake(BooleanSupplier retract) { // untested
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
                  .until(elevator.above(ElevatorSubsystem.MIN_EMPTY_GROUND_INTAKE)),
              groundArm
                  .moveToPosition(GroundArm.GROUND_POSITION)
                  .withDeadline(Commands.waitUntil(intakeSensor.inIntake().or(retract))),
              groundArm.moveToPosition(GroundArm.STOWED_POSITION),
              groundSpinny.setFunnelIntakePower(),
              coralPreIntake())
          .unless(clawFull)
          .withName("Ground Intake");
    }
  }

  public Command coralStow() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_STOWED),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_STOWED),
            spinnyClaw.stop())
        .withName("Coral Stow");
  }

  public Command coralPreIntake() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.CORAL_PRE_INTAKE),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN),
            spinnyClaw.coralRejectPower())
        .andThen(Commands.print("end of preIntake()"))
        .withName("PreIntake");
  }

  public Trigger inCoralPreIntakePosition() {
    return new Trigger(
        () ->
            elevator.atPosition(ElevatorSubsystem.CORAL_PRE_INTAKE)
                && armPivot.atPosition(ArmPivot.CORAL_PRESET_DOWN));
  }

  public Command coralIntake() {
    return Commands.sequence(
            untilClawFull(
                Commands.sequence(
                    Commands.parallel(
                        spinnyClaw.coralIntakePower(),
                        armPivot.moveToPosition(ArmPivot.CORAL_PRESET_DOWN)),
                    elevator.setLevel(ElevatorSubsystem.CORAL_INTAKE_POS))),
            spinnyClaw.stop(),
            elevator.setLevel(ElevatorSubsystem.CORAL_STOWED),
            coralStow())
        .withName("Coral Intake");
  }

  public Command algaeLevelThreeFourIntake() {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR)))
        .withName("Algae L3-L4 Intake");
  }

  public Command algaeLevelTwoThreeIntake() {
    return Commands.sequence(
            Commands.parallel(
                spinnyClaw.algaeIntakePower(),
                armPivot.moveToPosition(ArmPivot.ALGAE_REMOVE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_TWO_THREE)))
        .withName("Algae L2-L3 Intake");
  }

  public Command algaeGroundIntake() {
    return Commands.parallel(
            spinnyClaw.algaeIntakePower(),
            Commands.sequence(
                armPivot.moveToPosition(ArmPivot.ALGAE_GROUND_INTAKE),
                elevator.setLevel(ElevatorSubsystem.ALGAE_GROUND_INTAKE)))
        .withName("Algae Ground Intake");
  }

  public Command algaeStow() {
    return Commands.parallel(
            elevator.setLevel(ElevatorSubsystem.ALGAE_STOWED),
            armPivot.moveToPosition(ArmPivot.ALGAE_STOWED),
            spinnyClaw.algaeGripIntakePower())
        .deadlineFor(colorSet(255, 255, 255, "White - Stowed").asProxy())
        .withName("Algae Stow");
  }

  public Command algaeProcessorScore(BooleanSupplier score) {
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

  public Command algaeNetScore(BooleanSupplier score) {
    return Commands.sequence(
            Commands.parallel(
                elevator.setLevel(ElevatorSubsystem.ALGAE_NET_SCORE),
                armPivot.moveToPosition(ArmPivot.ALGAE_NET_SCORE),
                spinnyClaw.algaeIntakePower()),
            Commands.waitUntil(score),
            spinnyClaw.algaeHoldExtakePower().withTimeout(0.7),
            Commands.waitSeconds(0.7),
            armPivot.moveToPosition(ArmPivot.CORAL_PRESET_UP),
            elevator.setLevel(ElevatorSubsystem.ALGAE_LEVEL_THREE_FOUR))
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
