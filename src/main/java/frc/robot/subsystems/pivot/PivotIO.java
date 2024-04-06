package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double velocityDegsPerSec = 0;
    public double positionDegs = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double positionSetpointDegs;

    public boolean gyroConnected = false;
    public double pitch = 0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setBrakeMode(boolean bool) {}

  public default void setPositionSetpointDegs(double positionDegs, double ffVolts) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
