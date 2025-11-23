package frc.robot.subsystems.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class Routines {

  public Routines() {}

  private AutoFactory autoFactory = Robot.getInstance().autoFactory;

  public Command runRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("I_K_Far_Right_Blue");
    AutoTrajectory preloadScore = routine.trajectory("finish");
    AutoTrajectory coralStation = routine.trajectory("YSM");
    AutoTrajectory secondScore = routine.trajectory("station");
    AutoTrajectory secondStation = routine.trajectory("K to Station");
    AutoTrajectory thirdScore = routine.trajectory("Station to L");
    AutoTrajectory thirdStation = routine.trajectory("L to Station");
    AutoTrajectory fourthScore = routine.trajectory("Station to A");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                preloadScore.resetOdometry(),
                preloadScore.cmd(),
                Commands.print("preloadScore done"),
                AutoLogic.scoreCommand(),
                Commands.print("Scoring done"),
                coralStation.cmd().alongWith(AutoLogic.readyIntakeCommand()),
                Commands.print("coralStation and readying intake done"),
                AutoLogic.isCollected(),
                Commands.print("collection done"),
                secondScore.cmd().alongWith(AutoLogic.intakeCommand()),
                Commands.print("secondScore and intaking done"),
                AutoLogic.scoreCommand(),
                Commands.print("scoring done!"),
                 secondStation.cmd().alongWith(AutoLogic.readyIntakeCommand()), Commands.print("secondStation and readied intake done"), AutoLogic.isCollected(),
                 Commands.print("second collection done"),thirdScore.cmd().alongWith(AutoLogic.intakeCommand()),
                 Commands.print("third transferred to intake"), AutoLogic.scoreCommand(), Commands.print("thirdscore done"),
                thirdStation.cmd().alongWith(AutoLogic.readyIntakeCommand()), Commands.print("third statino and readied intake"), AutoLogic.isCollected(),
                Commands.print("third collection done"),
                fourthScore.cmd().alongWith(AutoLogic.intakeCommand()), Commands.print("fourth in intake"), AutoLogic.scoreCommand(),  
                Commands.print("fourthscore done"), Commands.print("ROUTINE DONE!")
                
                ));
    return routine.cmd();
  }
}
