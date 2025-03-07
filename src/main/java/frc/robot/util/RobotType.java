package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public enum RobotType {
  BONK(0x38, 0xd9, 0x93),
  CRANE(0x22, 0xb0, 0x92),
  COMPETITION(0, 0, 0),
  TESTBASE(0x33, 0x9d, 0xe7);

  @SuppressWarnings("ImmutableEnumChecker")
  private final MACAddress macAddress;

  RobotType(int a, int b, int c) {
    macAddress = MACAddress.of(a, b, c);
  }

  private static RobotType detectType() {
    for (RobotType robotType : RobotType.values()) {
      if (robotType.macAddress.exists()) {
        return robotType;
      }
    }
    DriverStation.reportWarning(
        "Code running on unknown MAC Address! Running competition code anyways", false);
    return COMPETITION;
  }

  private static RobotType currentRobotType;

  public static RobotType getCurrent() {
    if (currentRobotType == null) {
      currentRobotType = detectType();
    }
    return currentRobotType;
  }
}
