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

   // private final ElevatorPivotIOInputsAutoLogged pInputs = new ElevatorPivotIOInputsAutoLogged();
   // private final ElevatorIOInputsAutoLogged eInputs = new ElevatorPivotIOInputsAutoLogged();
   



  public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO elevator ){

    this.pivot = pivot;
    this.elevator = elevator;

  }

  public void setPosition(double pos1, double pos2){
     pivot.setPosition(pos1);
     elevator.setPosition(pos2);
       
     
  }

 public void setVelocity(double vel1, double vel2){
    pivot.setVelocity(vel1);
    elevator.setVelocity(vel2);
     
 }

 public void stopMotors(){
    pivot.stop();
    elevator.stop();
 
 }

 public double findAngle(double c){


   return (c % 2048) * 360;

}
public double findDistance(double a){


  return Constants.SHOOTER_LENGTH * Math.cos(findAngle(a));
}

public double findHeight(double a){

  return Constants.SHOOTER_LENGTH * Math.sin(findAngle(a));
}

public double findAcceptableDeviation(double h, double d, double a){
 double acceptableTheta =0;
 
  h = findHeight(a);
  double y = Math.sqrt( Math.pow((78.263 - h), 2) + Math.pow(d,2));
  double x = (78.263 - (4.8 + h + (78.263 - h))) + 4.8;

   acceptableTheta = Math.atan2(y, x);
  return acceptableTheta;
}


 @Override
 public void periodic() {
  //  pivot.updateInputs(pInputs);
  //  elevator.updateInputs(eInputs);

  //  Logger.processInputs("pivot Motor", pInputs);
  //  Logger.processInputs("elevate motor", eInputs);

 }







}
