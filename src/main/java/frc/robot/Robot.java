// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SubsystemConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutonomousField;
import frc.robot.util.BuildInfo;
import frc.robot.util.MatchTab;
import frc.robot.util.RobotType;

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
  public final SuperStructure superStructure;
  private final PowerDistribution PDH;

  Mechanism2d mech;
  MechanismRoot2d root;
  MechanismLigament2d m_elevator;
  MechanismLigament2d m_wrist;

  protected Robot() {
    // non public for singleton. Protected so test class can subclass

    instance = this;
    robotType = RobotType.getCurrent();
    CanBridge.runTCP();
    PDH = new PowerDistribution(Hardware.PDH_ID, ModuleType.kRev);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(PDH);
    mech = new Mechanism2d(1, 2);
    root = mech.getRoot("climber", 0.5 + Units.inchesToMeters(5.5), Units.inchesToMeters(19.5));
    SmartDashboard.putData("Mechanism", mech);
    m_elevator =
        root.append(new MechanismLigament2d("elevator", 1, 90, 2, new Color8Bit(Color.kRed)));
    var pivot =
        m_elevator.append(
            new MechanismLigament2d(
                "pivot offset", Units.inchesToMeters(4), -90, 2, new Color8Bit(Color.kDarkRed)));
    m_wrist =
        pivot.append(
            new MechanismLigament2d(
                "wrist", Units.inchesToMeters(14.5), 270, 6, new Color8Bit(Color.kFirstRed)));

    sensors = new Sensors();
    subsystems = new Subsystems(sensors);
    if (SubsystemConstants.DRIVEBASE_ENABLED) {
      AutoBuilderConfig.buildAuto(subsystems.drivebaseSubsystem);
    }
    if (SubsystemConstants.ELEVATOR_ENABLED
        && SubsystemConstants.ARMPIVOT_ENABLED
        && SubsystemConstants.SPINNYCLAW_ENABLED) {
      superStructure =
          new SuperStructure(
              subsystems.elevatorSubsystem,
              subsystems.armPivotSubsystem,
              subsystems.spinnyClawSubsytem,
              subsystems.groundArm,
              subsystems.groundSpinny,
              subsystems.elevatorLEDSubsystem,
              sensors.armSensor,
              sensors.branchSensors,
              sensors.intakeSensor);
    } else {
      superStructure = null;
    }
    controls = new Controls(subsystems, sensors, superStructure);

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
            (command, interruptor) ->
                System.out.println(
                    "Command interrupted: "
                        + command.getName()
                        + "; Cause: "
                        + interruptor.map(cmd -> cmd.getName()).orElse("<none>")));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

    SmartDashboard.putData(CommandScheduler.getInstance());
    BuildInfo.logBuildInfo();

    if (SubsystemConstants.DRIVEBASE_ENABLED) {
      AutoLogic.registerCommands();
      AutonomousField.initShuffleBoard("Field", 0, 0, this::addPeriodic);
      AutoLogic.initShuffleBoard();
      FollowPathCommand.warmupCommand().schedule();
    }
    MatchTab.create(sensors, subsystems);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (subsystems.visionSubsystem != null) {
      subsystems.visionSubsystem.update();
    }
    m_elevator.setLength(subsystems.elevatorSubsystem.getHeightMeters());
    m_wrist.setAngle(subsystems.armPivotSubsystem.getAngle());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    // on the end of diabling, make sure all of the motors are set to break and wont move upon
    // enabling
    if (subsystems.drivebaseSubsystem != null) {
      subsystems.drivebaseSubsystem.brakeMotors();
    }
    if (subsystems.climbPivotSubsystem != null) {
      subsystems.climbPivotSubsystem.brakeMotors();
      subsystems.climbPivotSubsystem.moveCompleteTrue();
    }
    if (subsystems.elevatorSubsystem != null) {
      subsystems.elevatorSubsystem.brakeMotors();
    }
  }

  @Override
  public void autonomousInit() {
    Shuffleboard.startRecording();
    if (SubsystemConstants.DRIVEBASE_ENABLED && AutoLogic.getSelectedAuto() != null) {
      AutoLogic.getSelectedAuto().schedule();
    }
    if (subsystems.climbPivotSubsystem != null) {
      subsystems.climbPivotSubsystem.moveCompleteFalse();
    }
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    Shuffleboard.startRecording();
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
