// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorPivotIOSim implements ElevatorPivotIO {
  private final DCMotor pivotGearbox = DCMotor.getFalcon500(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          pivotGearbox,
          5,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20), 0.1),
          Units.inchesToMeters(20),
          0,
          Math.PI,
          true,
          0);
  private final PIDController pid = new PIDController(0, 0, 0);

  private double currentAmps = 0.0;
  private double appliedVolts = 0.0;
  private double velocity = 0.0;
  private double position = 0.0;
  private double positionSetpoint = 0.0;

  @Override
  public void updateInputs(ElevatorPivotIOInputs inputs) {
    positionSetpoint = pid.getSetpoint();

    appliedVolts += MathUtil.clamp(pid.calculate(sim.getAngleRads(), positionSetpoint), -12.0, 12);

    sim.setInputVoltage(appliedVolts);
    // sim.setInput(appliedVolts);

    position = sim.getAngleRads();
    velocity = sim.getVelocityRadPerSec();
    currentAmps = sim.getCurrentDrawAmps();

    inputs.positionSetpoint = positionSetpoint;
    inputs.appliedVolts = appliedVolts;
    inputs.pivotPosition = position;
    inputs.pivotVelocity = velocity;
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
    pid.setSetpoint(sim.getAngleRads());
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
