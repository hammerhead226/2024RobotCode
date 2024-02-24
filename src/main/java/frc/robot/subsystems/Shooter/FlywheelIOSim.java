// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getKrakenX60(2), 1.5, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double velocitySetpoint = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.leftVelocitySetpoint = velocitySetpoint;

    inputs.leftVelocityRPM = sim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = appliedVolts;
    inputs.leftCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocityRPM(double leftVelocity, double rightVelocity, double ffVolts) {
    closedLoop = true;
    this.velocitySetpoint = leftVelocity;
    pid.setSetpoint(leftVelocity);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    velocitySetpoint = 0;
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
