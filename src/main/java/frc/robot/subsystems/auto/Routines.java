package frc.robot.subsystems.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;

public class Routines {


  public Routines() {

  }
  private AutoFactory autoFactory = Robot.getInstance().autoFactory;
  private Controls controls = Robot.getInstance().controls;
  private Subsystems subsystems = Robot.getInstance().subsystems;
    public Command runRoutine() {
  AutoRoutine routine = autoFactory.newRoutine("test");
    AutoTrajectory preloadScore =   routine.trajectory("finish");
    AutoTrajectory coralStation =   routine.trajectory("YSM");
    AutoTrajectory secondScore = routine.trajectory("station");
    routine.active().onTrue(Commands.sequence(preloadScore.resetOdometry(),  
    preloadScore.cmd(), AutoAlign.autoAlign(subsystems.drivebaseSubsystem, Robot.getInstance().controls, AutoAlign.AlignType.ALLB),
    coralStation.cmd(),  AutoLogic.isCollected(),
    secondScore.cmd(),
    AutoAlign.autoAlign(subsystems.drivebaseSubsystem, controls, AutoAlign.AlignType.ALLB)));
 return routine.cmd();
}


}
