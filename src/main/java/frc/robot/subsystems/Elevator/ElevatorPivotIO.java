package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorPivotIO {
    @AutoLog
    public static class ElevatorPivotIOInputs {
        double pivotAbsolutePosition = 0;
        double pivotVelocity = 0;
        double pivotPosition = 0;
        double currentAmps = 0;
        double appliedVolts = 0;
    }
    public default void updateInputs(ElevatorPivotIOInputs inputs) {}
    
    public default void setPosition(double position) {}

    public default void setVelocity(double velocity) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}
    
}
