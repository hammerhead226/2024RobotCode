package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface AmpBarIO {

  @AutoLog
  public static class AmpBarIOInputs {

    public double barVelocityDegsPerSec = 0;
    public double barPositionDegrees = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double barPositionSetpointDegrees = 0;
  }

  public default void updateInputs(AmpBarIOInputs inputs) {}

  public default void setBrakeMode(boolean bool) {}

  public default void setPositionSetpoint(double position, double ffVolts) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
