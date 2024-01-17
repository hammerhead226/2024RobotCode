package frc.robot.subsystems.Elevator;

import java.util.logging.LogRecord;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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



 @Override
 public void periodic() {
  //  pivot.updateInputs(pInputs);
  //  elevator.updateInputs(eInputs);

  //  Logger.processInputs("pivot Motor", pInputs);
  //  Logger.processInputs("elevate motor", eInputs);

 }







}
