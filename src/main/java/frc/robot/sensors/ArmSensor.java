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
  // VALUES ARE IN METERES
  private static final double TROUGH_LOWER_LIMIT = 0.15;
  private static final double TROUGH_UPPER_LIMIT = 0.25;
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
          if (getSensorDistance().in(Meters) > TROUGH_LOWER_LIMIT
              && getSensorDistance().in(Meters) < TROUGH_UPPER_LIMIT) {
            return true;
          } else {
            return false;
          }
        });
  }

  public Trigger inClaw() {
    return new Trigger(
        () -> {
          if (getSensorDistance().in(Meters) > CLAW_LOWER_LIMIT
              && getSensorDistance().in(Meters) < CLAW_UPPER_LIMIT) {
            return true;
          } else {
            return false;
          }
        });
  }
}
