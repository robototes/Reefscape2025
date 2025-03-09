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

public class ArmSensor {

  private final LaserCan mainSensor;
  // VALUES ARE IN METERS
  private static final double TROUGH_LOWER_LIMIT = 0.18;
  private static final double TROUGH_UPPER_LIMIT = 0.28;
  private static final double CLAW_LOWER_LIMIT = 0.06;
  private static final double CLAW_UPPER_LIMIT = 0.08;

  public ArmSensor() {
    mainSensor = new LaserCan(Hardware.MAIN_ARM_SENSOR);
    ConfigureSensor(mainSensor);
    ShuffleboardTab tab = Shuffleboard.getTab("ArmSensor");
    tab.addBoolean("inTrough", inTrough());
    tab.addBoolean("inClaw", inClaw());
    tab.addDouble("Distance(m)", () -> getSensorDistance().in(Meters));
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

  public Distance getSensorDistance() {
    LaserCan.Measurement measurement = mainSensor.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return Millimeter.of(measurement.distance_mm);
    } else {
      return Millimeter.of(-1);
    }
  }

  public Trigger inTrough() {
    return new Trigger(
            () -> {
              double distance = getSensorDistance().in(Meters);
              return distance > TROUGH_LOWER_LIMIT && distance < TROUGH_UPPER_LIMIT;
            })
        .debounce(0.5);
  }

  public Trigger inClaw() {
    return new Trigger(
            () -> {
              double distance = getSensorDistance().in(Meters);
              return distance > CLAW_LOWER_LIMIT && distance < CLAW_UPPER_LIMIT;
            })
        .debounce(0.1);
  }
}
