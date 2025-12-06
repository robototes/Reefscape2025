package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GainsTuningEntry {

  private final NetworkTable table;
  private String kSubsystemName;
  private NetworkTableEntry kPEntry;
  private NetworkTableEntry kIEntry;
  private NetworkTableEntry kDEntry;
  private NetworkTableEntry kAEntry;
  private NetworkTableEntry kVEntry;
  private NetworkTableEntry kSEntry;
  private NetworkTableEntry kGEntry;

  /** Base PID Feedback entires. Excludes kA, kV, kS, and kG
   *
   */
  public GainsTuningEntry(String subsystemName, double kp, double ki, double kd) {
    table = NetworkTableInstance.getDefault().getTable("Gains/" + subsystemName);
    this.kSubsystemName = subsystemName;
    kPEntry = table.getEntry("kP");
    kIEntry = table.getEntry("kI");
    kDEntry = table.getEntry("kD");
    kAEntry = null;
    kVEntry = null;
    kSEntry = null;
    kGEntry = null;
    kPEntry.setDouble(kp);
    kIEntry.setDouble(ki);
    kDEntry.setDouble(kd);
  }

  /** With ALL Feedback + Feedforward entries, INCLUDING kA
   *
   */
  public GainsTuningEntry(
      String subsystemName,
      double kp,
      double ki,
      double kd,
      double ka,
      double kv,
      double ks,
      double kg) {
    this(subsystemName, kp, ki, kd);
    kAEntry = table.getEntry("kA");
    kVEntry = table.getEntry("kV");
    kSEntry = table.getEntry("kS");
    kGEntry = table.getEntry("kG");
    kAEntry.setDouble(ka);
    kVEntry.setDouble(kv);
    kSEntry.setDouble(ks);
    kGEntry.setDouble(kg);
  }

  /** With Feedback + Feedforward entries, except kA
   *
   */
  public GainsTuningEntry(
      String subsystemName, double kp, double ki, double kd, double kv, double ks, double kg) {
    this(subsystemName, kp, ki, kd);
    kVEntry = table.getEntry("kV");
    kSEntry = table.getEntry("kS");
    kGEntry = table.getEntry("kG");
    kVEntry.setDouble(kv);
    kSEntry.setDouble(ks);
    kGEntry.setDouble(kg);
  }

  public String getSubsystem() {
    return kSubsystemName;
  }

  /** Get proportional gain (P)
   * 
   */
  public double getP() {
    return kPEntry.getDouble(0);
  }

    /** Get integral gain (I)
     * 
     */
  public double getI() {
    return kIEntry.getDouble(0);
  }

    /** Get derivative gain (D)
     * 
     */
  public double getD() {
    return kDEntry.getDouble(0);
  }

    /** Get acceleration gain (A)
     * 
     */
  public double getA() {
    return kAEntry != null ? kAEntry.getDouble(0) : 0;
  }

    /** Get velocity gain (V)
     * 
     */
  public double getV() {
    return kVEntry != null ? kVEntry.getDouble(0) : 0;
  }

    /** Get static gain (S)
     * 
     */
  public double getS() {
    return kSEntry != null ? kSEntry.getDouble(0) : 0;
  }

    /** Get gravity gain (G)
     * 
     */
  public double getG() {
    return kGEntry != null ? kGEntry.getDouble(0) : 0;
  }

  /** Check if any gains have changed using the current values from the subsystem.
   * Compares all gains: P, I, D, A, V, S, G
   */
  public boolean anyGainsChanged(
      double currentP,
      double currentI,
      double currentD,
      double currentA,
      double currentV,
      double currentS,
      double currentG) {

    return (pidffGainsChanged(currentP, currentI, currentD, currentV, currentS, currentG)
        || currentA != getA());
  }

  /** Check if any gains have changed using the current values from the subsystem.
   * Compares P, I, D, V, S, G
   */
  public boolean pidffGainsChanged(
      double currentP,
      double currentI,
      double currentD,
      double currentV,
      double currentS,
      double currentG) {

    return (pidGainsChanged(currentP, currentI, currentD)
        || currentV != getV()
        || currentS != getS()
        || currentG != getG());
  }

  /** Check if any gains have changed using the current values from the subsystem.
   * Compares P, I, D only
   */
  public boolean pidGainsChanged(double currentP, double currentI, double currentD) {

    return (currentP != getP() || currentI != getI() || currentD != getD());
  }
}
