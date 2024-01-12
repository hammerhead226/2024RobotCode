package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotIOInputs {
        public double intakePosition = 0.0;
        public double intakeVelocity = 0.0;

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(IntakePivotIOInputs inputs) {}
    
    public default void setPosition(double position) {}

    public default void setVelocity(double velocity) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}
}