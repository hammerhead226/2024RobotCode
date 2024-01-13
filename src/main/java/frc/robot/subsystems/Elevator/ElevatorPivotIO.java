package frc.robot.subsystems.Elevator;

public interface ElevatorPivotIO {
    @AutoLog
    public static class ElevatorPivotIOInputs {
        public double pivotPosition = 0.0;
        public double pivotVelocity = 0.0;
        public double appliedVolts = 0.0;

        public double currentAmps = 0.0;
    }

    public default void updateInputs(ElevatorPivotIOInputs inputs) {}
    
    public default void setPosition(double position) {}

    public default void setVelocity(double veloicty) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}
    
}
