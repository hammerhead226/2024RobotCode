// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO pivot;
  private final PivotIOInputsAutoLogged pInputs = new PivotIOInputsAutoLogged();

  private static double kP;
  private static double kG;
  private static double kV;

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile.Constraints pivotConstraints;

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrent = new TrapezoidProfile.State();

  double currTarget;
  double lastTarget;

  private ArmFeedforward pivotFFModel;

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    this.pivot = pivot;
    switch (Constants.getMode()) {
      case REAL:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      case REPLAY:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        ;
        break;
      case SIM:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      default:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
    }

    maxVelocityDegPerSec = 150;
    maxAccelerationDegPerSecSquared = 226;
    // maxAccelerationDegPerSecSquared = 180;

    pivotConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    pivotProfile = new TrapezoidProfile(pivotConstraints);

    setPivotGoal(90);
    pivotCurrent = pivotProfile.calculate(0, pivotCurrent, pivotGoal);

    pivot.configurePID(kP, 0, 0);
    pivotFFModel = new ArmFeedforward(0, kG, kV, 0);
  }

  public double getPivotPositionDegs() {
    return pInputs.positionDegs;
  }

  public boolean atSetpoint() {
    return (Math.abs(getPivotError()) <= Constants.PivotConstants.THRESHOLD);
  }

  private double getPivotError() {
    return pInputs.positionSetpointDegs - pInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    pivot.setPositionSetpointDegs(
        positionDegs,
        pivotFFModel.calculate(Math.toRadians(positionDegs), Math.toRadians(velocityDegsPerSec)));
  }

  public void pivotStop() {
    pivot.stop();
  }

  public void setPivotGoal(double setpoint) {
    pivotGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  @Override
  public void periodic() {
    pivot.updateInputs(pInputs);

    pivotCurrent = pivotProfile.calculate(Constants.LOOP_PERIOD_SECS, pivotCurrent, pivotGoal);

    setPositionDegs(pivotCurrent.position, pivotCurrent.velocity);

    Logger.processInputs("Pivot", pInputs);
    Logger.recordOutput("pivot error", getPivotError());
    // This method will be called once per scheduler run
  }
}
