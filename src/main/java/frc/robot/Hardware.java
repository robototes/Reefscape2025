package frc.robot;

public class Hardware {
  // Add motor IDs here

  // Swerve: 1-12

  // elevator [20-24]
  public static final int ELEVATOR_MOTOR_ONE = 20;
  public static final int ELEVATOR_MOTOR_TWO = 21;

  // branch sensors [25-29] (mounted on elevator)
  public static final int BRANCH_SENSOR_LEFT = 25;
  public static final int BRANCH_SENSOR_RIGHT = 26;

  // arm pivot [30-34]
  public static final int ARM_PIVOT_MOTOR_ID = 30;

  // vision
  public static final String PHOTON_IP = "10.24.12.3";
  public static final String FRONT_CAM = "Arducam_OV9281B";
  public static final String BACK_CAM = "Arducam_OV9281A";

  // arm Sensors [35-39]
  public static final int MAIN_ARM_SENSOR = 35;
}
