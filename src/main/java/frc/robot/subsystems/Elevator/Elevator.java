package frc.robot.subsystems.Elevator;

import java.util.logging.LogRecord;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorPivotIO.ElevatorPivotIOInputs;

public class Elevator extends SubsystemBase {
    private final ElevatorPivotIO pivot;

    private final ElevatorExtenderIO elevator;

    private final ElevatorPivotIOInputsAutoLogged pInputs = new ElevatorPivotIOInputsAutoLogged();
    private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();
   



  public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO elevator ){

    this.pivot = pivot;
    this.elevator = elevator;

  }

  public void setPositionElevator( double pos){
     
     elevator.setPosition(pos);
       
     
  }
  public void setPositionPivot(double pos){
    pivot.setPosition(pos);

  }

 public void setVelocity(double vel1, double vel2){
    pivot.setVelocity(vel1);
    elevator.setVelocity(vel2);
     
 }

 public void stopMotors(){
    pivot.stop();
    elevator.stop();
 
 }

 public double findAngle(double ticks){


   return (ticks % 2048) * 360;

}


public double findDistance(double angle){


  return Constants.SHOOTER_LENGTH * Math.cos(findAngle(angle));
}

public double findHeight(double angle){

  return (Constants.SHOOTER_LENGTH * Math.sin(findAngle(angle))) + Constants.PIVOT_HEIGHT;
}

public double findAverageAngle(double currentPos, double distance){
    
   
   double shooterHeight = findHeight(findAngle(currentPos)) + Constants.PIVOT_HEIGHT;
   double hP = 83.063 - (9.6 + shooterHeight);
   double minAngle = Math.atan2(hP, distance- 17.791);
   double maxAngle = Math.atan2(hP + 9.6, distance - 17.791);
   double averageAngle = (minAngle + maxAngle) / 2;
   
   return averageAngle;
  

   
}

public double convertAnglesToTicks(double angle){

   return (angle / 360) * 2048;
}
 @Override
 public void periodic() {
    pivot.updateInputs(pInputs);
    elevator.updateInputs(eInputs);

   Logger.processInputs("pivot Motor", pInputs);
   Logger.processInputs("elevate motor", eInputs);

 }







}
