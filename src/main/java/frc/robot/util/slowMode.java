package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;

public class slowMode {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("SlowMode");
  private NetworkTableEntry slowModeEntry;

  public slowMode() {
    slowModeEntry = table.getEntry("slowMode");
    slowModeEntry.setBoolean(false);
  }

  public BooleanSupplier isSlowMode() {
    return () -> slowModeEntry.getBoolean(true);
  }

  public double slowFactor() {
    return (isSlowMode().getAsBoolean()) ? 0.1 : 1.0;
  }
}
