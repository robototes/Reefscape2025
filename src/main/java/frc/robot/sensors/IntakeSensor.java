package frc.robot.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GroundArmConstants;

public class IntakeSensor {

  private final LaserCan intakeSensor;
  // VALUES ARE IN METERS
  private static final double INTAKE_LOWER_LIMIT = 0.005;
  private static final double INTAKE_UPPER_LIMIT = 0.05;

  public IntakeSensor() {
    intakeSensor = new LaserCan(GroundArmConstants.GROUND_INTAKE_SENSOR);
    ConfigureSensor(intakeSensor);
    ShuffleboardTab tab = Shuffleboard.getTab("IntakeSensor");
    tab.addBoolean("In Intake", inIntake());
    tab.addDouble("Distance(m)", () -> getSensorDistance().in(Meters));
  }

  private void ConfigureSensor(LaserCan Sensor) {
    try {
      Sensor.setRangingMode(LaserCan.RangingMode.SHORT);
      Sensor.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 16, 16));
      Sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration Failed! " + e);
    }
  }

  public Distance getSensorDistance() {
    LaserCan.Measurement measurement = intakeSensor.getMeasurement();
    if (measurement == null) {
      return Millimeter.of(-1);
    }
    if (measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      if (measurement.status != LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS) {
        return Millimeter.of(-2);
      } else {
        return Millimeter.of(-3);
      }
    }
    return Millimeter.of(measurement.distance_mm);
  }

  public Trigger inIntake() {
    return new Trigger(
            () -> {
              double distance = getSensorDistance().in(Meters);
              return distance > INTAKE_LOWER_LIMIT && distance < INTAKE_UPPER_LIMIT;
            })
        .debounce(0.1);
  }
}
