package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controls {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int OPERATOR_CONTROLLER_PORT = 1;

  @SuppressWarnings("UnusedVariable")
  private final CommandXboxController driverController;

  @SuppressWarnings("UnusedVariable")
  private final CommandXboxController operatorController;

  @SuppressWarnings("UnusedVariable")
  private final Subsystems s;

  public Controls(Subsystems s) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    this.s = s;
  }
}
