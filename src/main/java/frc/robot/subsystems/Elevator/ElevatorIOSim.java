// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor simGearbox = DCMotor.getFalcon500(2);
  private ElevatorSim sim = new ElevatorSim(simGearbox, 1, 1, 0.01, 0.0, 3, true, 0.0);
  private PIDController pid = new PIDController(0, 0, 0);

  private double position = 0.0;
  private double velocity = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  private double positionSetpoint = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    positionSetpoint = pid.getSetpoint();

    appliedVolts +=
        MathUtil.clamp(pid.calculate(sim.getPositionMeters(), positionSetpoint), -12.0, 12);

    sim.setInputVoltage(appliedVolts);

    position = sim.getPositionMeters();
    velocity = sim.getVelocityMetersPerSecond();
    currentAmps = sim.getCurrentDrawAmps();

    inputs.positionSetpoint = positionSetpoint;
    inputs.appliedVolts = appliedVolts;
    inputs.elevatorPosition = position;
    inputs.elevatorVelocity = velocity;
    inputs.currentAmps = currentAmps;

    sim.update(Constants.LOOP_PERIOD_SECS);
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    appliedVolts = ffVolts;
    pid.setSetpoint(position);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    pid.setSetpoint(sim.getPositionMeters());
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
