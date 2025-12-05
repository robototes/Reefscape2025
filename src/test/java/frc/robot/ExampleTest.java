package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;
import java.util.function.DoubleSupplier;

class ExampleTest {

  @Test
  void testMocking() {
    // Create a mock object
    DoubleSupplier mockSupplier = mock(DoubleSupplier.class);

    // Define behavior
    when(mockSupplier.getAsDouble()).thenReturn(10.0);

    // Verify behavior
    assertEquals(10.0, mockSupplier.getAsDouble(), 0.001);
  }

  @Test
  void testBasicAssertion() {
    assertEquals(2, 1 + 1);
  }
}
