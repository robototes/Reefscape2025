package frc.robot;

public class Hardware {
  // Add motor IDs here

  // Swerve: 1-12

  // LEDs [15-19]
  public static final int ELEVATOR_LED = 15;

  // elevator [20-24]
  public static final int ELEVATOR_MOTOR_ONE = 20;
  public static final int ELEVATOR_MOTOR_TWO = 21;

  // branch sensors [25-29] (mounted on elevator)
  // fixed it for u connor ;) - mech
  public static final int BRANCH_SENSOR_LEFT = 25;
  public static final int BRANCH_SENSOR_RIGHT = 26;

  // arm pivot [30-34]
  public static final int ARM_PIVOT_MOTOR_ID = 30;
  public static final int ARM_PIVOT_CANDI_ID = 31;

  // climb [50-59]
  public static final int CLIMB_PIVOT_MOTOR_ONE_ID = 50;
  public static final int CLIMB_PIVOT_MOTOR_TWO_ID = 51;
  public static final int CLIMB_PIVOT_CANDI_ID = 52;

  // climb DIO
  public static final int CLIMB_SENSOR = 1;

  // arm Sensors [35-39]
  public static final int MAIN_ARM_SENSOR = 35;

  // scoring [40 - 44]
  public static final int SPINNY_CLAW_MOTOR_ID = 40;

  // vision
  public static final String PHOTON_IP = "10.24.12.11";
  public static final String LEFT_CAM = "Arducam_OV9782L";
  public static final String RIGHT_CAM = "Arducam_OV9281R";
}
