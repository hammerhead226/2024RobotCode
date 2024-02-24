// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO pivot;
  private final PivotIOInputsAutoLogged pInputs = new PivotIOInputsAutoLogged();

  private static final LoggedTunableNumber pivotkP = new LoggedTunableNumber("elevatorPivotkP");

  private final TrapezoidProfile pivotProfile;
  private final TrapezoidProfile.Constraints pivotConstraints =
      new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 3);

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrent = new TrapezoidProfile.State();

  private final ArmFeedforward pivotFFModel;

  private final SysIdRoutine sysId;

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    this.pivot = pivot;
    switch (Constants.currentMode) {
      case REAL:
        pivotFFModel = new ArmFeedforward(0, 0, 0);
        pivotkP.initDefault(0);
        break;
      case REPLAY:
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(0);
        break;
      case SIM:
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(0);
        break;
      default:
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  pivot.runCharacterization(voltage.in(Volts));
                },
                null,
                this));

    setPivotGoal(30);
    pivotProfile = new TrapezoidProfile(pivotConstraints);

    pivotCurrent = pivotProfile.calculate(0, pivotCurrent, pivotGoal);

    pivot.configurePID(pivotkP.get(), 0, 0);
  }

  public double getPivotPosition() {
    return pInputs.pivotPosition;
  }

  public boolean pivotAtSetpoint() {
    return (Math.abs(getPivotError()) <= Constants.PivotConstants.THRESHOLD);
  }

  private double getPivotError() {
    return pInputs.positionSetpoint - pInputs.pivotPosition;
  }

  public void setPositionPivot(double position, double velocity) {
    pivot.setPositionSetpointDegs(position, pivotFFModel.calculate(position, velocity));
  }

  public void pivotStop() {
    pivot.stop();
  }

  public void setPivotGoal(double setpoint) {
    pivotGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    pivot.updateInputs(pInputs);

    pivotCurrent = pivotProfile.calculate(Constants.LOOP_PERIOD_SECS, pivotCurrent, pivotGoal);

    setPositionPivot(pivotCurrent.position, pivotCurrent.velocity);

    Logger.processInputs("Elevator Pivot", pInputs);
    if (pivotkP.hasChanged(hashCode())) {
      pivot.configurePID(pivotkP.get(), 0, 0);
    }
    // This method will be called once per scheduler run
  }
}
