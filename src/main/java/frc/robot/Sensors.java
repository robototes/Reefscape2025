package frc.robot;

import static frc.robot.Sensors.SensorConstants.*;

import frc.robot.sensors.ArmSensor;

public class Sensors {
     public static class SensorConstants {
    // <SENSOR>_ENABLED constants go here
    public static final boolean ARMSENSOR_ENABLED = true;
  }

  // Sensors go here
    public final ArmSensor armSensor;

  public Sensors() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (ARMSENSOR_ENABLED) {
      armSensor = new ArmSensor();
    } else {
      armSensor = null;
    }
  }
}
