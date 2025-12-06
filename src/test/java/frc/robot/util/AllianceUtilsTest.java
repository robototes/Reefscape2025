package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mockStatic;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

class AllianceUtilsTest {

  private MockedStatic<DriverStation> mockedDriverStation;

  @BeforeEach
  void setUp() {
    mockedDriverStation = mockStatic(DriverStation.class);
  }

  @AfterEach
  void tearDown() {
    mockedDriverStation.close();
  }

  @Test
  void testIsBlue() {
    mockedDriverStation.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Blue));

    assertTrue(AllianceUtils.isBlue());
    assertFalse(AllianceUtils.isRed());
  }

  @Test
  void testIsRed() {
    mockedDriverStation.when(DriverStation::getAlliance).thenReturn(Optional.of(Alliance.Red));

    assertTrue(AllianceUtils.isRed());
    assertFalse(AllianceUtils.isBlue());
  }

  @Test
  void testNoAlliance() {
    mockedDriverStation.when(DriverStation::getAlliance).thenReturn(Optional.empty());

    assertFalse(AllianceUtils.isBlue());
    assertFalse(AllianceUtils.isRed());
  }
}
