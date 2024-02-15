package frc.robot.util;

import frc.robot.Constants;

public class Calculations {
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

   /*    
   public double findAverageAngle(double currentPosition, double distance) {
      double shooterHeight = findHeight(convertTicksToDegrees(currentPosition)) + Constants.PIVOT_HEIGHT;
      double heightPerfect = 83.063 - (9.6 + shooterHeight);
      double minAngle = Math.atan2(heightPerfect, distance - 17.791);
      double maxAngle = Math.atan2(heightPerfect + 9.6, distance - 17.791);
      double averageAngle = (minAngle + maxAngle) / 2;
   
      return averageAngle;
   }
   */

   // This method takes distance in inches and returns angle in ticks
   // This method has calculations based on test shooter 
   // Update the method to equations from real robot
   public double calculateNeededAngle (double distance) {
    double angleErrorScope = 0.5;

    double highestAngle = ((-0.21 * distance) + 67.44);
    double desiredShootingAngle = (highestAngle - angleErrorScope);
    
    return (convertAnglesToTicks(desiredShootingAngle));
 }

}
