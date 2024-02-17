package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocity = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocity, double ffVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
