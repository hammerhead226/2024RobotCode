package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorExtenderIO {
    @AutoLog
    public static class ElevatorIOInputs{
        double elevatorPosition = 0;
        double elevatorVelocity = 0;
        double currentAmps = 0;
        double appliedVolts = 0;
    }
        
    public default void updateInputs(ElevatorIOInputs inputs) {}
    
    public default void setPosition(double position) {}

    public default void setVelocity(double veloicty) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}
}
