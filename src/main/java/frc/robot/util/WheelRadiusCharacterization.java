package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import java.text.DecimalFormat;
import java.text.NumberFormat;

public class WheelRadiusCharacterization {
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 4; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // Rad/Sec^2
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  CompTunerConstants.FrontLeft.LocationX, CompTunerConstants.FrontLeft.LocationY),
              Math.hypot(
                  CompTunerConstants.FrontRight.LocationX,
                  CompTunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  CompTunerConstants.BackLeft.LocationX, CompTunerConstants.BackLeft.LocationY),
              Math.hypot(
                  CompTunerConstants.BackRight.LocationX, CompTunerConstants.BackRight.LocationY)));
  private static final NumberFormat FORMATTER = new DecimalFormat("#0.000");
  private static final int NUM_MODULES = 4;

  // get the wheel rotation positions of the swerve modules
  public static double[] getWheelRadiusCharacterizationPositions(CommandSwerveDrivetrain drive) {
    return drive.getWheelRotations();
  }

  public static Command wheelRadiusCharacterizationCommand(CommandSwerveDrivetrain drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      SwerveRequest driveRequest =
                          new SwerveRequest.ApplyRobotSpeeds()
                              .withSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                      drive.setControl(driveRequest);
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = getWheelRadiusCharacterizationPositions(drive);
                      state.lastAngle = drive.getState().Pose.getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = drive.getState().Pose.getRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = getWheelRadiusCharacterizationPositions(drive);
                          double wheelDelta = 0.0;
                          for (int i = 0; i < NUM_MODULES; i++) {
                            wheelDelta +=
                                Math.abs(positions[i] - state.positions[i]) / (double) NUM_MODULES;
                          }
                          wheelDelta *= 2 * Math.PI;
                          double wheelRadius = (state.gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;

                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + FORMATTER.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + FORMATTER.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + FORMATTER.format(wheelRadius)
                                  + " meters, "
                                  + FORMATTER.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        .withName("wheel radius characterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[NUM_MODULES];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
