// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ElevatorPivotIOSim implements ElevatorPivotIO {
  private final DCMotor pivotGearbox = DCMotor.getFalcon500(1);

  private final SingleJointedArmSim sim = new SingleJointedArmSim(pivotGearbox, 1, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20), 2), Units.inchesToMeters(20), 0, Math.PI, true, 0);
  private final PIDController pid = new PIDController(0.5, 0, 0);
  private double velocity = 0.0;
  private double position = 0.0;


  public void updateInputs(ElevatorPivotIOInputs inputs) {

    sim.update(0.02);

    inputs.pivotPosition = Math.toDegrees(sim.getAngleRads());
    inputs.pivotVelocity = Units.radiansPerSecondToRotationsPerMinute(sim.getVelocityRadPerSec());
    // inputs.appliedVolts = motor.get() * RobotController.getBatteryVoltage();
    inputs.currentAmps = sim.getCurrentDrawAmps(); 

  }

  public void setPosition(double position) {
    this.position = position;
   sim.setState(position, velocity);
  }

  public void setVelocity(double velocity) {
    this.velocity = velocity;
    sim.setState(position, velocity);
  }


  public void stop() {
    sim.setState(position, velocity);
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
