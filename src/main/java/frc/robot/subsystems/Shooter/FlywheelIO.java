package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double leftVelocityRPM = 0;
    public double leftAppliedVolts = 0;
    public double leftCurrentAmps = 0;
    public double leftVelocitySetpoint = 0.0;

    public double rightVelocityRPM = 0;
    public double rightAppliedVolts = 0;
    public double rightCurrentAmps = 0;
    public double rightVelocitySetpoint = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocityRPM(double leftVelocity, double rightVelocity, double ffVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
