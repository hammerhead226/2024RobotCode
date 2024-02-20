package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorGyroIO {
   @AutoLog
    public static class ElevatorGyroIOInputs{
          
        public boolean isConnected = false;
        public double rotation = 0;

    }

    public default void updateInputs(ElevatorGyroIOInputs inputs) {}

}