package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtils {
  public static boolean isBlue() {
    if (!DriverStation.getAlliance().isEmpty()) {
      return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);
    }
    return false;
  }

  public static boolean isRed() {
    if (!DriverStation.getAlliance().isEmpty()) {
      return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }
    return false;
  }
}
