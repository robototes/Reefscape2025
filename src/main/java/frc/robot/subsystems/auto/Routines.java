package frc.robot.subsystems.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class Routines {


  public Routines() {

  }
  private AutoFactory autoFactory = Robot.getInstance().autoFactory;
    public Command runRoutine() {
  AutoRoutine routine = autoFactory.newRoutine("test");
    AutoTrajectory preloadScore =   routine.trajectory("finish");
    AutoTrajectory coralStation =   routine.trajectory("YSM");
    AutoTrajectory secondScore = routine.trajectory("station");
    routine.active().onTrue(Commands.sequence(preloadScore.resetOdometry(),  
    preloadScore.cmd(), Commands.print("preloadScore done"), AutoLogic.scoreCommand(), Commands.print("Scoring done"),
    coralStation.cmd().alongWith(AutoLogic.readyIntakeCommand()), Commands.print("coralStation and readying intake done"),  AutoLogic.isCollected(), 
    Commands.print("collection done"),
    secondScore.cmd().alongWith(AutoLogic.intakeCommand()), Commands.print("secondScore and intaking done"),
AutoLogic.scoreCommand(), Commands.print("scoring done!"), Commands.print("routine done!")
));
 return routine.cmd();
}


}
