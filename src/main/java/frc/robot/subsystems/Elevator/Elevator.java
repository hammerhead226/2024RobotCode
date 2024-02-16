package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PolynomialRegression;


public class Elevator extends SubsystemBase {
   private final ElevatorPivotIO pivot;
   private final ElevatorExtenderIO elevator;

   private final ElevatorPivotIOInputsAutoLogged pInputs = new ElevatorPivotIOInputsAutoLogged();
   private final ElevatorExtenderIOInputsAutoLogged eInputs = new ElevatorExtenderIOInputsAutoLogged();
   
   private final PolynomialRegression angleCalculations = new PolynomialRegression(Constants.xDataPoints, Constants.yDataPoints, 1, "DistanceInInches");
   
   public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO elevator ){
      this.pivot = pivot;
      this.elevator = elevator;
   }

   public void setPositionElevator(double position) {
      elevator.setPosition(position);
   }

   public void setPositionPivot(double position) {
      pivot.setPosition(position);
   }

   public void setPivotVelocity(double pivotVelocity) {
      pivot.setVelocity(pivotVelocity);
   }

   public void setElevatorVelocity(double elevatorVelocity) {
      pivot.setVelocity(elevatorVelocity);
   }

   public void pivotStop(){
      pivot.stop();
   }

   public void elevatorStop(){
      elevator.stop();
   }

   public double convertTicksToDegrees(double ticks){
      return (ticks % 2048) * (360/2048);
   }

   public double convertAnglesToTicks(double angle){
      return (angle % 360) * 2048/360;
   }

   public double findDistance(double angle){
      return Constants.SHOOTER_LENGTH * Math.cos(convertTicksToDegrees(angle));
   }

   public double findHeight(double angle){
      return (Constants.SHOOTER_LENGTH * Math.sin(convertTicksToDegrees(angle))) + Constants.PIVOT_HEIGHT;
   }

   @Override
   public void periodic() {
      pivot.updateInputs(pInputs);
      elevator.updateInputs(eInputs);

      Logger.processInputs("pivot Motor", pInputs);
      Logger.processInputs("elevate motor", eInputs);
   }
}
