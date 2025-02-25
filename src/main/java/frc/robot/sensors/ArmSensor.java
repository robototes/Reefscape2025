package frc.robot.sensors;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import java.util.function.BooleanSupplier;

public class ArmSensor {

  private final LaserCan mainSensor;
  // Sensor reads in meters, is converted to milimeters in code?
  private static final double TROUGH_LOWER_LIMIT = 250; // untested guess value (in millimeters)
  private static final double TROUGH_UPPER_LIMIT = 350; // untested guess value
  private static final double CLAW_LOWER_LIMIT = 30; // untested guess value
  private static final double CLAW_UPPER_LIMIT = 130; // untested guess value

  public ArmSensor() {
    mainSensor = new LaserCan(Hardware.MAIN_ARM_SENSOR);
    ConfigureSensor(mainSensor);
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
          if (getSensorDistance().in(Millimeters) > TROUGH_LOWER_LIMIT
              && getSensorDistance().in(Millimeters) < TROUGH_UPPER_LIMIT) {
            return true;
          } else {
            return false;
          }
        });
  }

  public BooleanSupplier inClaw() {
    return () -> {
      if (getSensorDistance().in(Millimeters) > CLAW_LOWER_LIMIT
          && getSensorDistance().in(Millimeters) < CLAW_UPPER_LIMIT) {
        return true;
      } else {
        return false;
      }
    };
  }
}
