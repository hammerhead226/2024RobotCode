// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeRollerIO roller;

  private final IntakeRollerIOInputsAutoLogged rInputs = new IntakeRollerIOInputsAutoLogged();

  public Intake(IntakeRollerIO roller) {
    this.roller = roller;

  }


  public void runRollers(double velocity){
     roller.setVelocity(velocity);

  }

  public void stopRollers(){
    roller.stop();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roller.updateInputs(rInputs);

    Logger.processInputs("roller Motor", rInputs);
  }
}
