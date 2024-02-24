package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {

    public double feederVelocityRPM = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double velocitySetpoint = 0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setVelocityRPM(double velocity, double ffVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
