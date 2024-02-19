package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorPivotIO pivot;
  private final ElevatorExtenderIO extender;

  private final ElevatorPivotIOInputsAutoLogged pInputs = new ElevatorPivotIOInputsAutoLogged();
  private final ElevatorExtenderIOInputsAutoLogged eInputs =
      new ElevatorExtenderIOInputsAutoLogged();

  private static final LoggedTunableNumber pivotkP = new LoggedTunableNumber("elevatorPivotkP");
  private static final LoggedTunableNumber extenderkP =
      new LoggedTunableNumber("elevatorExtenderkP");

  private final TrapezoidProfile.Constraints pivotConstraints =
      new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 3);
  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrent = new TrapezoidProfile.State();

  private final TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(1, 0.7);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private final ElevatorFeedforward elevatorFFModel;
  private final ArmFeedforward pivotFFModel;

  public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO extender) {
    this.pivot = pivot;
    this.extender = extender;

    switch (Constants.currentMode) {
      case REAL:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(Constants.ElevatorConstants.PIVOT_PID[0]);
        extenderkP.initDefault(Constants.ElevatorConstants.EXTENDER_PID[0]);
        break;
      case REPLAY:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(0);
        extenderkP.initDefault(15);
        break;
      case SIM:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(0);
        extenderkP.initDefault(15);
        break;
      default:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        pivotFFModel = new ArmFeedforward(0, 0.4, 0.7);
        pivotkP.initDefault(0);
        extenderkP.initDefault(15);
        break;
    }
    pivotkP.initDefault(0);
    extenderkP.initDefault(15);

    this.pivot.configurePID(pivotkP.get(), 0, 0);
    this.extender.configurePID(extenderkP.get(), 0, 0);
  }

  public double getPivotPosition() {
    return pInputs.pivotPosition;
  }

  public double getExtenderPosition() {
    return eInputs.elevatorPosition;
  }

  private double getPivotError() {
    return pInputs.positionSetpoint - pInputs.pivotPosition;
  }

  private double getExtenderError() {
    return eInputs.positionSetpoint - eInputs.elevatorPosition;
  }

  public boolean pivotAtSetpoint() {
    return (Math.abs(getPivotError()) <= Constants.ElevatorConstants.PIVOT_THRESHOLD);
  }

  public boolean extenderAtSetpoint() {
    return (Math.abs(getExtenderError()) <= Constants.ElevatorConstants.EXTENDER_THRESHOLD);
  }

  public void setPivotGoal(double setpoint) {
    pivotGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setExtenderGoal(double setpoint) {
    extenderGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setPositionExtend(double position, double velocity) {
    extender.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
  }

  public void setPositionPivot(double position, double velocity) {
    pivot.setPositionSetpoint(position, pivotFFModel.calculate(position, velocity));
  }

  public void pivotStop() {
    pivot.stop();
  }

  public void elevatorStop() {
    extender.stop();
  }

  public double calculateAngle() {
    double angle = 0.0;
    return angle;
  }

  @Override
  public void periodic() {
    pivot.updateInputs(pInputs);
    extender.updateInputs(eInputs);

    TrapezoidProfile pivotProfile = new TrapezoidProfile(pivotConstraints);
    TrapezoidProfile extenderProfile = new TrapezoidProfile(extenderConstraints);

    extenderCurrent =
        extenderProfile.calculate(Constants.LOOP_PERIOD_SECS, extenderCurrent, extenderGoal);
    pivotCurrent = pivotProfile.calculate(Constants.LOOP_PERIOD_SECS, pivotCurrent, pivotGoal);

    setPositionPivot(pivotCurrent.position, pivotCurrent.velocity);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator Pivot", pInputs);
    Logger.processInputs("Elevator Extender", eInputs);

    if (extenderkP.hasChanged(hashCode())) {
      extender.configurePID(extenderkP.get(), 0, 0);
    }

    if (pivotkP.hasChanged(hashCode())) {
      pivot.configurePID(pivotkP.get(), 0, 0);
    }
  }
}
