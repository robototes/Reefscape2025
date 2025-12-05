package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mockStatic;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

class AllianceUtilsTest {

  @Test
  void testIsBlue() {
    try (MockedStatic<DriverStation> mockedDriverStation = mockStatic(DriverStation.class)) {
      mockedDriverStation
          .when(DriverStation::getAlliance)
          .thenReturn(Optional.of(Alliance.Blue));

      assertTrue(AllianceUtils.isBlue());
      assertFalse(AllianceUtils.isRed());
    }
  }

  @Test
  void testIsRed() {
    try (MockedStatic<DriverStation> mockedDriverStation = mockStatic(DriverStation.class)) {
      mockedDriverStation
          .when(DriverStation::getAlliance)
          .thenReturn(Optional.of(Alliance.Red));

      assertTrue(AllianceUtils.isRed());
      assertFalse(AllianceUtils.isBlue());
    }
  }

  @Test
  void testNoAlliance() {
    try (MockedStatic<DriverStation> mockedDriverStation = mockStatic(DriverStation.class)) {
      mockedDriverStation
          .when(DriverStation::getAlliance)
          .thenReturn(Optional.empty());

      assertFalse(AllianceUtils.isBlue());
      assertFalse(AllianceUtils.isRed());
    }
  }
}
