// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIOTalonFX shooterMotor1;
  private final ShooterIOTalonFX shooterMotor2;

  //private final ShooterIOInputsAutoLogged sInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIOTalonFX shooterMotor1, ShooterIOTalonFX shooterMotor2) {

     this.shooterMotor1 = shooterMotor1;
     this.shooterMotor2 = shooterMotor2;

  }


  public void stopMotors(){

    shooterMotor1.stop();
    shooterMotor2.stop();
  }

  public void setVelocity(double velocity){

    shooterMotor1.setVelocity(velocity);
    shooterMotor2.setVelocity(velocity);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    //shooterMotor1.updateInputs(sInputs);

    //Logger.processInputs("shooter motors", sInputs);
  }
}
