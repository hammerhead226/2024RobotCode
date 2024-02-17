package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorPivotIO {
  @AutoLog
  public static class ElevatorPivotIOInputs {
    public double pivotAbsolutePosition = 0;
    public double pivotVelocity = 0;
    public double pivotPosition = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
  }

  public default void updateInputs(ElevatorPivotIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
