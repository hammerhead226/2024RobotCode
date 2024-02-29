// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO pivot;
  private final PivotIOInputsAutoLogged pInputs = new PivotIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD");

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/kA");

  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Pivot/MaxVelocity");
  private static final LoggedTunableNumber maxAccelerationDegPerSecSquared =
      new LoggedTunableNumber("Pivot/MaxAcceleration");

  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile.Constraints pivotConstraints;

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrent = new TrapezoidProfile.State();

  private ArmFeedforward pivotFFModel;

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    this.pivot = pivot;
    switch (Constants.getMode()) {
      case REAL:
        kS.initDefault(0.1);
        kG.initDefault(0.3);
        kV.initDefault(0.03);
        kA.initDefault(0);

        kP.initDefault(0.4);
        kI.initDefault(0.04);
        kD.initDefault(0);
        break;
      case REPLAY:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        break;
      case SIM:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(1.8);
        kI.initDefault(0.7);
        kD.initDefault(0);
        break;
      default:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        break;
    }

    maxVelocityDegPerSec.initDefault(80);
    maxAccelerationDegPerSecSquared.initDefault(110);

    pivotConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityDegPerSec.get(), maxAccelerationDegPerSecSquared.get());
    pivotProfile = new TrapezoidProfile(pivotConstraints);

    setPivotGoal(45 + 59);
    pivotCurrent = pivotProfile.calculate(0, pivotCurrent, pivotGoal);

    pivot.configurePID(kP.get(), kI.get(), kD.get());
    pivotFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  public double getPivotPositionDegs() {
    return pInputs.positionDegs;
  }

  public boolean pivotAtSetpoint() {
    return (Math.abs(getPivotError()) <= Constants.PivotConstants.THRESHOLD);
  }

  private double getPivotError() {
    return pInputs.positionSetpointDegs - pInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    pivot.setPositionSetpointDegs(
        positionDegs, pivotFFModel.calculate(positionDegs, velocityDegsPerSec));
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

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pivot.configurePID(kP.get(), kI.get(), kD.get());
    }

    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      pivotFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSecSquared.hasChanged(hashCode())) {
      pivotConstraints =
          new TrapezoidProfile.Constraints(
              maxVelocityDegPerSec.get(), maxAccelerationDegPerSecSquared.get());
      pivotProfile = new TrapezoidProfile(pivotConstraints);
    }
    // This method will be called once per scheduler run
  }
}
