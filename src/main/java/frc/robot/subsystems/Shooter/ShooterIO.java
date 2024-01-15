package frc.robot.subsystems.Shooter;

public interface ShooterIO {
    //@AutoLog
    public static class ShooterIOInputs{
       public double shooterVelocity = 0;
       public double appliedAmps = 0;
       public double currentAmps = 0;

    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVelocity(double velocity) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}

}
