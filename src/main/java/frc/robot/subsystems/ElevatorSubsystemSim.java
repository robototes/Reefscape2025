package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;

/** Simulation logic for the ElevatorSubsystem. */
public class ElevatorSubsystemSim {
  // Simulation PID constants - tuned for smooth motion
  private static final double SIM_KP = 50.0;
  private static final double SIM_KD = 5.0;
  private static final double MOTOR_ROTATIONS_PER_METER = 19.68;

  private final ElevatorSim m_elevatorSim;
  private final TalonFXSimState m_motorOneSimState;
  private final TalonFXSimState m_motorTwoSimState;
  private final StructPublisher<Pose3d> m_posePublisher;
  public DoubleTopic arm_Pivot = NetworkTableInstance.getDefault().getDoubleTopic("Arm Pivot");
  private double simTargetHeightMeters = 0.0;
  private double simPreviousError = 0.0;
  final DoubleSubscriber dblSub;
  /**
   * Constructor for the elevator simulation. Creates all simulation objects internally.
   *
   * @param motor1 First elevator motor (leader)
   * @param motor2 Second elevator motor (follower)
   * @param posePublisher Publisher for visualization pose
   */
  public ElevatorSubsystemSim(
      TalonFX motor1, TalonFX motor2, StructPublisher<Pose3d> posePublisher) {
    // Get sim states from motors
    this.m_motorOneSimState = motor1.getSimState();
    this.m_motorTwoSimState = motor2.getSimState();
    this.m_posePublisher = posePublisher;
    dblSub = arm_Pivot.subscribe(0.0);
    // Initialize elevator simulation
    // Elevator specs: ~38 rotations = 1.93 meters (38 / 19.68)
    this.m_elevatorSim =
        new ElevatorSim(
            DCMotor.getFalcon500(2), // 2 Falcon 500 motors
            1.0, // Gearing ratio (adjust based on actual mechanism)
            5.0, // Carriage mass in kg (adjust based on actual mass)
            0.02, // Drum radius in meters (adjust for your spool/pulley)
            0.0, // Min height in meters
            1.93, // Max height in meters (~38 rotations / 19.68)
            true, // Simulate gravity
            0.0 // Starting height in meters
            );
  }

  /**
   * Set the target position for the simulated elevator.
   *
   * @param targetPositionRotations Target position in motor rotations
   */
  public void setTargetPosition(double targetPositionRotations) {
    simTargetHeightMeters = targetPositionRotations / MOTOR_ROTATIONS_PER_METER;
  }

  /**
   * Update the simulation. Should be called periodically.
   *
   * @param currentPositionRotations Current position reading from motor (for visualization)
   */
  public void updateSimulation(double currentPositionRotations) {
    // Calculate PD control to smoothly move to target position
    double currentHeightMeters = m_elevatorSim.getPositionMeters();
    double error = simTargetHeightMeters - currentHeightMeters;
    double errorRate = (error - simPreviousError) / 0.02; // derivative

    // PD control output
    double pidOutput = (SIM_KP * error) + (SIM_KD * errorRate);
    simPreviousError = error;

    // Apply voltage to simulation
    m_elevatorSim.setInputVoltage(pidOutput);
    m_elevatorSim.update(0.02); // 20ms periodic cycle

    // Get simulated position and convert to motor rotations
    double simHeightMeters = m_elevatorSim.getPositionMeters();
    double simPositionRotations = simHeightMeters * MOTOR_ROTATIONS_PER_METER;

    // Update motor sim states with realistic position
    m_motorOneSimState.setRawRotorPosition(simPositionRotations);
    m_motorTwoSimState.setRawRotorPosition(simPositionRotations);

    // Update visualization
    updateVisualization(currentPositionRotations);
  }

  /**
   * Update the 3D pose visualization for the elevator.
   *
   * @param currentPositionRotations Current position in motor rotations
   */
  private void updateVisualization(double currentPositionRotations) {
    double bottomZ = 0.2;
    double topZ = 1.55;
    double minPos = 0.0;
    double maxPos = 37.5;
    double targetZ =
        (bottomZ + ((currentPositionRotations - minPos) / (maxPos - minPos)) * (topZ - bottomZ));
double wristPos = dblSub.get();
//System.out.println("Wrist pose is:" + wristPos)
    m_posePublisher.set(new Pose3d(0.2, 0.0, targetZ, new Rotation3d(Units.degreesToRadians(-90), wristPos, Units.degreesToRadians(-90) )));
  }
}
