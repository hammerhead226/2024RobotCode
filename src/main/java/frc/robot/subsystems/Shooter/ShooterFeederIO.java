package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFeederIO {
  @AutoLog
  public static class ShooterFeederIOInputs {

    public double feederVelocity = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
  }

  public default void updateInputs(ShooterFeederIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
