package frc.robot.sensors;

import static edu.wpi.first.units.Units.Millimeter;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Hardware;

public class BranchSensors {

  private final LaserCan leftSensor;
  private final LaserCan rightSensor;

  public BranchSensors() {
    leftSensor = new LaserCan(Hardware.BRANCH_SENSOR_LEFT);
    rightSensor = new LaserCan(Hardware.BRANCH_SENSOR_RIGHT);
    ConfigureSensor(leftSensor);
    ConfigureSensor(rightSensor);
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

  public Distance getLeftSensorDistance() {
    LaserCan.Measurement measurement = leftSensor.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return Millimeter.of(measurement.distance_mm);
    } else {
      return Millimeter.of(-1);
    }
  }

  public Distance getRightSensorDistance() {
    LaserCan.Measurement measurement = rightSensor.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return Millimeter.of(measurement.distance_mm);
    } else {
      return Millimeter.of(-1);
    }
  }
}
