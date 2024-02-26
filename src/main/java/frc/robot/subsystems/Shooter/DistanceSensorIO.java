package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface DistanceSensorIO {
  @AutoLog
  public static class DistanceSensorIOInputs {
    public boolean hasNote = false;
    public boolean isConnected = false;
  }

  public default void updateInputs(DistanceSensorIOInputs inputs) {}

  public default void setAutomaticMode(boolean enabled) {}
}
