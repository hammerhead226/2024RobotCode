// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ElevatorPivotIOSim implements ElevatorPivotIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotor simMotor = DCMotor.getFalcon500(1);
  private SingleJointedArmSim sim = new SingleJointedArmSim(simMotor, 0, 0, 0, 0, 0, true, 0);
  private PIDController pid = new PIDController(0, 0, 0);


  @Override
  public void updateInputs(ElevatorPivotIOInputs inputs) {
    sim.update(LOOP_PERIOD_SECS);

    // finish the pivotAbsolutePosition and appliedVolts variables for logging
    inputs.pivotAbsolutePosition = ;
    inputs.pivotVelocity = sim.getVelocityRadPerSec();
    inputs.pivotPosition = sim.getAngleRads();
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.appliedVolts = ;
  }

  // rewrite this method
  @Override
  public void setPosition(double position) {

  }

  // rewrite this method
  @Override
  public void setVelocity(double velocity) {

  }

  // rewrite this method
  @Override
  public void stop() {

  }

  // rewrite this method
  @Override
  public void setVoltage(double voltage) {

  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
