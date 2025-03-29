package frc.robot.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;

public class BranchSensors {

  private final LaserCan leftSensor;
  private final LaserCan rightSensor;

  // temporary
  private static final double RIGHT_SENSOR_MIN = 0.0;
  private static final double RIGHT_SENSOR_MAX = 0.0;
  private static final double LEFT_SENSOR_MAX = 0.0;
  private static final double LEFT_SENSOR_MIN = 0.0;

  public BranchSensors() {
    leftSensor = new LaserCan(Hardware.BRANCH_SENSOR_LEFT);
    rightSensor = new LaserCan(Hardware.BRANCH_SENSOR_RIGHT);
    ConfigureSensor(leftSensor);
    ConfigureSensor(rightSensor);
    ShuffleboardTab tab = Shuffleboard.getTab("BranchSensor");
    tab.addDouble("LeftDistance(m)", () -> getLeftSensorDistance().in(Meters));
    tab.addDouble("RightDistance(m)", () -> getRightSensorDistance().in(Meters));
  }

  private void ConfigureSensor(LaserCan Sensor) {
    try {
      Sensor.setRangingMode(LaserCan.RangingMode.SHORT);
      Sensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      Sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration Failed! " + e);
    }
  }

  private Distance getSensorDistance(LaserCan sensor) {
    LaserCan.Measurement measurement = sensor.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return Millimeter.of(measurement.distance_mm);
    } else {
      return Millimeter.of(10000);
    }
  }

  public Distance getLeftSensorDistance() {
    return getSensorDistance(leftSensor);
  }

  public Distance getRightSensorDistance() {
    return getSensorDistance(rightSensor);
  }

  // APPROVED BY SENSEI WU LETSS GOOOOOOOO :D
  public Trigger withinScoreRange() {
    return new Trigger(
            () -> {
              double rightDistance = getRightSensorDistance().in(Meters);
              double leftDistance = getLeftSensorDistance().in(Meters);
              return (LEFT_SENSOR_MIN < leftDistance && leftDistance < LEFT_SENSOR_MAX)
                  && (RIGHT_SENSOR_MIN < rightDistance && rightDistance < RIGHT_SENSOR_MAX);
            })
        .debounce(0.5);
  }
}
