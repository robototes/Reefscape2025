package frc.robot;

import static frc.robot.Sensors.SensorConstants.*;

import frc.robot.sensors.ArmSensor;
import frc.robot.sensors.BranchSensors;
import frc.robot.sensors.IntakeSensor;
import frc.robot.util.RobotType;

public class Sensors {
  public static class SensorConstants {
    // <SENSOR>_ENABLED constants go here
    public static final boolean ARMSENSOR_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean INTAKE_SENSOR_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
    public static final boolean BRANCHSENSORS_ENABLED =
        RobotType.getCurrent() == RobotType.COMPETITION && true;
  }

  // Sensors go here
  public final ArmSensor armSensor;
  public final IntakeSensor intakeSensor;
  public final BranchSensors branchSensors;

  public Sensors() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (ARMSENSOR_ENABLED) {
      armSensor = new ArmSensor();
    } else {
      armSensor = null;
    }

    if (INTAKE_SENSOR_ENABLED) {
      intakeSensor = new IntakeSensor();
    } else {
      intakeSensor = null;
    }

    if (BRANCHSENSORS_ENABLED) {
      branchSensors = new BranchSensors();
    } else {
      branchSensors = null;
    }
  }
}
