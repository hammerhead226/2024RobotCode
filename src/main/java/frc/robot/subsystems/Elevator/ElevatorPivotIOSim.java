// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorPivotIOSim implements ElevatorPivotIO{
  private ElevatorSim sim = new ElevatorSim;
  private PIDController pid = new PIDController(0, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double velocity = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorPivotIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getVelocityMetersPerSecond()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.pivotVelocity = sim.getVelocityMetersPerSecond();
    inputs.pivotPosition = sim.getPositionMeters();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double position) {
    closedLoop = true;
    sim.setState(position, velocity);
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoop = true;
    this.velocity = velocity;
    pid.setSetpoint(velocity);
  }
  
  @Override
  public void stop() {
    setVelocity(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
