// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorExtenderIOSim implements ElevatorExtenderIO {

  private final DCMotor simGearbox = DCMotor.getFalcon500(2);
  private ElevatorSim sim = new ElevatorSim(2, 1, simGearbox, 0.2, 3, true, 0.2);
  private PIDController pid = new PIDController(0.2, 0.2, 0.2);

  private double velocity = 0.0;
  private double position = 0.0;

  @Override
  public void updateInputs(ElevatorExtenderIOInputs inputs) {
    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.elevatorPosition = sim.getPositionMeters();
    inputs.elevatorVelocity = sim.getVelocityMetersPerSecond();
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double position) {
    this.position = position;
    sim.setState(position, velocity);
  }

  @Override
  public void setVelocity(double velocity) {
    this.velocity = velocity;
    sim.setState(position, velocity);
  }

  @Override
  public void stop() {
    sim.setState(position, velocity);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
