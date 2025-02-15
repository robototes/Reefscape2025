// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AutoLogic;
import frc.robot.util.AutonomousField;
import frc.robot.util.BuildInfo;
import frc.robot.util.RobotType;
import java.util.List;

public class Robot extends TimedRobot {
  /** Singleton Stuff */
  private static Robot instance = null;

  public static Robot getInstance() {
    if (instance == null) instance = new Robot();
    return instance;
  }

  private final RobotType robotType;
  public final Controls controls;
  public final Subsystems subsystems;
  public final Sensors sensors;
  public List<Pose2d> pathPoses;

  protected Robot() {
    // non public for singleton. Protected so test class can subclass

    instance = this;
    robotType = RobotType.getCurrent();

    LiveWindow.disableAllTelemetry();

    subsystems = new Subsystems();
    controls = new Controls(subsystems);
    sensors = new Sensors();

    SmartDashboard.putString("current bot", robotType.toString());
    SmartDashboard.putData("commandScheduler", CommandScheduler.getInstance());

    if (RobotBase.isReal()) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), true);
    }

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("Command initialized: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> System.out.println("Command interrupted: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

    SmartDashboard.putData(CommandScheduler.getInstance());

    BuildInfo.logBuildInfo();

    DriverStation.silenceJoystickConnectionWarning(true);
    AutoLogic.initShuffleBoard();
    AutonomousField.initShuffleBoard("Field", 0, 0, this::addPeriodic);

    AutoLogic.registerCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    AutoLogic.registerCommand();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    // Checks if FMS is attatched and enables joystick warning if true

    AutoLogic.RunAuto(subsystems.drivebaseSubsystem);
    Command autoCommand = AutoLogic.getAutoCommand(AutoLogic.getSelectedAutoName());

    if (autoCommand != null) {
      autoCommand.schedule();

    } else {
      DriverStation.reportError("Auto command not found!", false);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public RobotType getRobotType() {
    return robotType;
  }

  public boolean isCompetition() {
    return getRobotType() == RobotType.COMPETITION;
  }
}
