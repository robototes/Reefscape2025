// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MACAddress;

public class Robot extends TimedRobot {
  /** Singleton Stuff */
  private static Robot instance = null;

  public enum RobotType {
    COMPETITION,
    CRANE,
    BONK;
  }

  public static Robot getInstance() {
    if (instance == null) instance = new Robot();
    return instance;
  }

  private final RobotType robotType;
  public final Controls controls;
  public final Subsystems subsystems;

  public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x38, 0xd9, 0x9e);
  public static final MACAddress BONK_ADDRESS = MACAddress.of(0x33, 0x9D, 0xE7);
  public static final MACAddress CRANE_ADDRESS = MACAddress.of(0x22, 0xB0, 0x92);

  private static RobotType getTypeFromAddress() {
    if (CRANE_ADDRESS.exists()) return RobotType.CRANE;
    if (BONK_ADDRESS.exists()) return RobotType.BONK;
    if (!COMPETITION_ADDRESS.exists())
      DriverStation.reportWarning(
          "Code running on unknown MAC Address! Running competition code anyways", false);
    return RobotType.COMPETITION;
  }

  protected Robot() {
    // non public for singleton. Protected so test class can subclass
    instance = this;
    robotType = getTypeFromAddress();

    LiveWindow.disableAllTelemetry();

    subsystems = new Subsystems();
    controls = new Controls(subsystems);

    SmartDashboard.putString("current bot", robotType.toString());

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

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

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
