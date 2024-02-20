package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ElevatorGyroIOPigeon2 implements ElevatorGyroIO{

      // private final Pigeon2 pigeon2 = new Pigeon2(Constants.ElevatorConstants.ELEVATOR_PIGEON_PORT, "2")
       private final Pigeon2 pigeon2 = new Pigeon2(RobotMap.ElevatorIDs.GYRO, Constants.CANBUS);
       private final StatusSignal<Double> pitch = pigeon2.getRoll();
       
   public ElevatorGyroIOPigeon2() {
     pigeon2.getConfigurator().apply(new Pigeon2Configuration());
     pitch.setUpdateFrequency(100.0);
     pigeon2.optimizeBusUtilization();
   }

     @Override
  public void updateInputs(ElevatorGyroIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(pitch).equals(StatusCode.OK);
    inputs.rotation = pitch.getValueAsDouble();
    
  }
    
}