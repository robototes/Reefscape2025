package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Sensors;
import frc.robot.Subsystems;

public class MatchTab {
  public static void create(Sensors sensors, Subsystems subsystems) {
    var tab = Shuffleboard.getTab("Match");
    tab.addBoolean(
            "has arm sensor",
            () ->
                (sensors.armSensor != null && sensors.armSensor.getSensorDistance().in(Meters) > 0))
        .withSize(1, 1)
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kBooleanBox);
    tab.addBoolean(
            "has ground intake sensor",
            () ->
                (sensors.intakeSensor != null
                    && sensors.intakeSensor.getSensorDistance().in(Meters) > 0))
        .withSize(1, 1)
        .withPosition(1, 0)
        .withWidget(BuiltInWidgets.kBooleanBox);
    tab.addBoolean(
            "has left branch sensor",
            () ->
                (sensors.branchSensors != null
                    && sensors.branchSensors.getLeftSensorDistance().in(Meters) > 0))
        .withSize(1, 1)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);
    tab.addBoolean(
            "has right branch sensor",
            () ->
                (sensors.branchSensors != null
                    && sensors.branchSensors.getRightSensorDistance().in(Meters) > 0))
        .withSize(1, 1)
        .withPosition(1, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);

    tab.addBoolean(
            "has recent vision measurements",
            () ->
                (subsystems.visionSubsystem != null
                    && subsystems.visionSubsystem.getTimeSinceLastReading() < 10))
        .withSize(2, 2)
        .withPosition(3, 0)
        .withWidget(BuiltInWidgets.kBooleanBox);
  }
}
