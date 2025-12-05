package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
  public class armPivotConstants {
    // Presets
    public static final double ARMPIVOT_KP = 38.5; // previously 50
    public static final double ARMPIVOT_KI = 0;
    public static final double ARMPIVOT_KD = 0;
    public static final double ARMPIVOT_KS = 0.1;
    public static final double ARMPIVOT_KV = 0.69;
    public static final double ARMPIVOT_KG = 0.18;
    public static final double ARMPIVOT_KA = 0.0;
    // Preset positions for Arm with Coral
    public static final double ARMPIVOT_CORAL_PRESET_L1 = 0;
    public static final double ARMPIVOT_CORAL_PRESET_L2 = 0.13;
    public static final double ARMPIVOT_CORAL_PRESET_L3 = 0.13;
    public static final double ARMPIVOT_CORAL_PRESET_L4 = 0.0;
    public static final double ARMPIVOT_CORAL_PRESET_PRE_L4 = 1.0 / 16.0;
    public static final double ARMPIVOT_CORAL_PRESET_STOWED = 0.125;
    public static final double ARMPIVOT_CORAL_PRESET_OUT = 0;
    public static final double ARMPIVOT_CORAL_PRESET_UP = 0.25; // Pointing directly upwards
    public static final double ARMPIVOT_CORAL_PRESET_DOWN = -0.25;
    // Preset positions for Arm with Algae
    public static final double ARMPIVOT_CORAL_POST_SCORE = -0.15;
    public static final double ARMPIVOT_CORAL_QUICK_INTAKE = -0.07;
    public static final double ARMPIVOT_ALGAE_REMOVE_PREPOS = 0;
    public static final double ARMPIVOT_ALGAE_REMOVE = 0;
    public static final double ARMPIVOT_ALGAE_FLING = -0.08;
    public static final double ARMPIVOT_ALGAE_STOWED = -0.05;
    public static final double ARMPIVOT_ALGAE_PROCESSOR_SCORE = -0.05;
    public static final double ARMPIVOT_ALGAE_GROUND_INTAKE = -0.085;
    public static final double ARMPIVOT_ALGAE_NET_SCORE = 0.175; // untested - old value was 0.18

    // Other Presets
    public static final double ARMPIVOT_CORAL_PRESET_GROUND_INTAKE = 0;
    public static final double ARMPIVOT_HARDSTOP_HIGH = 0.32;
    public static final double ARMPIVOT_HARDSTOP_LOW = -0.26;
    public static final double ARMPIVOT_POS_TOLERANCE = Units.degreesToRotations(5);
    public static final double ARMPIVOT_PLACEHOLDER_CORAL_WEIGHT_KG = 0.8;
    // Constant for gear ratio (the power that one motor gives to gear)
    public static final double ARM_RATIO = (12.0 / 60.0) * (20.0 / 60.0) * (18.0 / 48.0);
    // arm pivot [30-34]
    public static final int ARM_PIVOT_MOTOR_ID = 30;
    public static final int ARM_PIVOT_CANDI_ID = 31;
    // arm Sensors [35-39]
    public static final int MAIN_ARM_SENSOR = 35;
  }

  public static class ClimbConstants {
    // various position and speed presets
    public static final double STOWED_MAX_PRESET = -0.447;
    public static final double STOWED_MIN_PRESET = -0.450;
    public static final double CLIMB_OUT_MAX_PRESET = -0.150;
    public static final double CLIMB_OUT_MIN_PRESET = -0.177;
    public static final double CLIMBED_MAX_PRESET = -0.325;
    public static final double CLIMBED_MIN_PRESET = -0.34;
    public static final double FORWARD_SOFT_STOP = -0.07;
    public static final double REVERSE_SOFT_STOP = -78;
    public static final double CLIMB_OUT_SPEED = 1.0;
    public static final double CLIMB_HOLD_STOWED = -0.001;
    public static final double CLIMB_HOLD_CLIMBOUT = -0.0;
    public static final double CLIMB_HOLD_CLIMBED = -0.0705;
    public static final double CLIMB_IN_SPEED = -0.75;
    public static final double CLIMB_STATOR_CURRENT_LIMIT = 150;
    public static final double CLIMB_SUPPLY_CURRENT_LIMIT = 75;

    public static final double climbInKp = 50;
    public static final double climbOutKp = 50;

    // positions for relation between motor encoder and WCP encoder
    // relative to eachother, likely not accurately zero'ed when obtained.
    public static final double MIN_ROTOR_POSITION = -50.45;
    public static final double MAX_ROTOR_POSITION = 14.456;
    public static final double MIN_ENCODER_POSITION = 0.611;
    public static final double MAX_ENCODER_POSITION = 0.915;

    // climb DIO
    public static final int CLIMB_SENSOR = 8;

    // climb [50-59]
    public static final int CLIMB_PIVOT_MOTOR_LEFT_ID = 50;
    public static final int CLIMB_PIVOT_MOTOR_RIGHT_ID = 51;
    public static final int CLIMB_PIVOT_CANCODER_ID = 52;
  }

  public class elevatorConstants {
    // Maximum is 38.34
    public static final double CORAL_LEVEL_FOUR_PRE_POS = 37.5;
    public static final double CORAL_LEVEL_FOUR_POS = 36;
    public static final double CORAL_LEVEL_THREE_PRE_POS = 18.65;
    public static final double CORAL_LEVEL_THREE_POS = 14;
    public static final double CORAL_LEVEL_TWO_PRE_POS = 6.94;
    public static final double CORAL_LEVEL_TWO_POS = 4.4;
    public static final double CORAL_LEVEL_ONE_POS = 4.2;
    public static final double ALGAE_LEVEL_TWO_THREE = 11;
    public static final double ALGAE_LEVEL_TWO_THREE_FLING = 16;
    public static final double ALGAE_LEVEL_THREE_FOUR = 21;
    public static final double ALGAE_LEVEL_THREE_FOUR_FLING = 25;
    public static final double ALGAE_STOWED = 2;
    public static final double ALGAE_PROCESSOR_SCORE = 2;
    public static final double ALGAE_NET_SCORE = 38; // untested
    public static final double ALGAE_GROUND_INTAKE = 0.05;
    public static final double CORAL_STOWED = CORAL_LEVEL_TWO_PRE_POS;
    public static final double CORAL_GROUND_INTAKE_POS = 7.2;
    public static final double CORAL_INTAKE_POS = 1.55;
    public static final double CORAL_PRE_INTAKE = 4.7;
    public static final double CORAL_QUICK_INTAKE = 1.6;
    public static final double MIN_EMPTY_GROUND_INTAKE = 4.5;
    public static final double MIN_FULL_GROUND_INTAKE = 8.0;
    public static final double MANUAL = 0.1;
    public static final double POS_TOLERANCE = 0.1;
    public static final double ELEVATOR_KP = 7.804;
    public static final double ELEVATOR_KI = 0;
    public static final double ELEVATOR_KD = 0.079221;
    public static final double ELEVATOR_KS = 0.33878;
    public static final double ELEVATOR_KV = 0.12975;
    public static final double ELEVATOR_KA = 0.0070325;
    public static final double REVERSE_SOFT_LIMIT = -0.05;
    public static final double FORWARD_SOFT_LIMIT = 38;
    public static final double UP_VOLTAGE = 5;
    public static final double DOWN_VOLTAGE = -3;
    public static final double HOLD_VOLTAGE = 0.6;
    public static final double CURRENT_STATOR_LIMIT = 160;
    public static final double CURRENT_SUPPLY_LIMIT = 80;

    // elevator zero button DIO
    public static final int ELEVATOR_ZERO_BUTTON = 0;
    // elevator [20-24]
    public static final int ELEVATOR_MOTOR_ONE = 20;
    public static final int ELEVATOR_MOTOR_TWO = 21;
  }

  public class groundArmConstants {
    public static final double ARMPIVOT_KP = 40;
    public static final double ARMPIVOT_KI = 0;
    public static final double ARMPIVOT_KD = 0;
    public static final double ARMPIVOT_KS = 0.9;
    public static final double ARMPIVOT_KV = 4;
    public static final double ARMPIVOT_KG = 0.048;
    public static final double ARMPIVOT_KA = 0;
    public static final double STOWED_POSITION = 0.46;
    public static final double UP_POSITION =
        0.27; // untested - should be somewhere in between stowed and ground
    public static final double GROUND_POSITION = -0.050;
    public static final double QUICK_INTAKE_POSITION = 0.31;
    public static final double POS_TOLERANCE = Units.degreesToRotations(5);
    public static final double GROUND_HOLD_VOLTAGE = -0.40;
    // ratio from motor rotations to output rotations
    public static final double ARM_RATIO = 60;
    // ground intake [45-49]
    public static final int GROUND_INTAKE_ARM_MOTOR = 45;
    public static final int GROUND_INTAKE_ARM_ENCODER = 47;
    public static final int GROUND_INTAKE_SENSOR = 48;
  }

  public class groundSpinnyConstants {
    public static final double GROUND_INTAKE_SPEED = -8;
    public static final double GROUND_INTAKE_JITTER_SPEED = 1;
    public static final double FUNNEL_INTAKE_SPEED = -2;
    public static final double QUICK_HANDOFF_EXTAKE_SPEED = 1;
    public static final double STATOR_CURRENT_STALL_THRESHOLD = 50;
    public static final int GROUND_INTAKE_SPINNY_MOTOR = 46;
  }

  public static class SpinnyClawConstants {
    public static final double CORAL_INTAKE_SPEED = -6;
    public static final double CORAL_EXTAKE_SPEED = 5;
    public static final double CORAL_REJECT_SPEED = 1;
    public static final double CORAL_L1_EXTAKE_SPEED = 1.7;
    public static final double ALGAE_INTAKE_SPEED = -4; // started at -3.5
    public static final double ALGAE_GRIP_INTAKE_SPEED = -3; // started at -2.5
    public static final double ALGAE_EXTAKE_SPEED = 14;
    public static final double ALGAE_PROCESSOR_EXTAKE_SPEED = 8;
    public static final double ALGAE_FLING_SPEED = 10;
    // spinny claw [40-44]
    public static final int SPINNY_CLAW_MOTOR_ID = 40;
  }

  public class visionConstants_Misc {
    // differences from center robot camera poses
    private static final double CAMERA_X_POS_METERS_LEFT = 0.26;
    private static final double CAMERA_X_POS_METERS_RIGHT = 0.27;
    private static final double CAMERA_Y_POS_METERS_LEFT = 0.25;
    private static final double CAMERA_Y_POS_METERS_RIGHT = -0.25;
    private static final double CAMERA_Z_POS_METERS_LEFT = 0.20;
    private static final double CAMERA_Z_POS_METERS_RIGHT = 0.21;
    private static final double CAMERA_ROLL_LEFT = Units.degreesToRadians(3);
    private static final double CAMERA_ROLL_RIGHT = Units.degreesToRadians(0.92);
    private static final double CAMERA_PITCH_LEFT = Units.degreesToRadians(-6.3);
    private static final double CAMERA_PITCH_RIGHT = Units.degreesToRadians(-8.3);
    private static final double CAMERA_YAW_LEFT = Units.degreesToRadians(-44.64);
    private static final double CAMERA_YAW_RIGHT = Units.degreesToRadians(46.42);
    // left camera diffrences from center robot
    public static final Transform3d ROBOT_TO_CAM_LEFT =
        new Transform3d(
            // Translation3d.kZero,
            CAMERA_X_POS_METERS_LEFT,
            CAMERA_Y_POS_METERS_LEFT,
            CAMERA_Z_POS_METERS_LEFT,
            // Rotation3d.kZero);
            new Rotation3d(CAMERA_ROLL_LEFT, CAMERA_PITCH_LEFT, CAMERA_YAW_LEFT));
    // right camera diffrences from center robot
    public static final Transform3d ROBOT_TO_CAM_RIGHT =
        new Transform3d(
            // Translation3d.kZero,
            CAMERA_X_POS_METERS_RIGHT,
            CAMERA_Y_POS_METERS_RIGHT,
            CAMERA_Z_POS_METERS_RIGHT,
            // Rotation3d.kZero);
            new Rotation3d(CAMERA_ROLL_RIGHT, CAMERA_PITCH_RIGHT, CAMERA_YAW_RIGHT));

    // Deviations
    public static final Vector<N3> STANDARD_DEVS =
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
    public static final Vector<N3> DISTANCE_SC_STANDARD_DEVS =
        VecBuilder.fill(1, 1, Units.degreesToRadians(50));
    public static final String PHOTON_IP = "10.24.12.11";
    public static final String LEFT_CAM = "Arducam_OV9782L";
    public static final String RIGHT_CAM = "Arducam_OV9281R";
    // branch sensors [25-29] (mounted on elevator)
    // fixed it for u connor ;) - mech
    public static final int BRANCH_SENSOR_LEFT = 25;
    public static final int BRANCH_SENSOR_RIGHT = 26;
    public static final int PDH_ID = 1;
    // Swerve: 1-12

    // LEDs [15-19]
    public static final int ELEVATOR_LED = 15;
  }
}
