package frc.robot;

import static frc.robot.Sensors.SensorConstants.*;

import frc.robot.sensors.ArmSensor;
import frc.robot.sensors.BranchSensors;

public class Sensors {
  public static class SensorConstants {
    // <SENSOR>_ENABLED constants go here
    public static final boolean ARMSENSOR_ENABLED = true;
    public static final boolean BRANCHSENSORS_ENABLED = false;
  }

  // Sensors go here
  public final ArmSensor armSensor;
  public final BranchSensors branchSensors;

  public Sensors() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (ARMSENSOR_ENABLED) {
      armSensor = new ArmSensor();
    } else {
      armSensor = null;
    }

    if (BRANCHSENSORS_ENABLED) {
      branchSensors = new BranchSensors();
    } else {
      branchSensors = null;
    }
  }
}
