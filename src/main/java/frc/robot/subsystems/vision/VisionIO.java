package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double angleDegs;
    public double distanceMeters;
    public double timeStamp;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
