package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Math.Conversions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorPivotIO pivot;
  private final ElevatorExtenderIO extender;

  private final ElevatorPivotIOInputsAutoLogged pInputs = new ElevatorPivotIOInputsAutoLogged();
  private final ElevatorExtenderIOInputsAutoLogged eInputs =
      new ElevatorExtenderIOInputsAutoLogged();

  private static final LoggedTunableNumber pivotkP = new LoggedTunableNumber("elevatorPivotkP");
  private static final LoggedTunableNumber extenderkP = new LoggedTunableNumber("elevatorExtenderkP");

  private final TrapezoidProfile.Constraints pivotConstraints = new TrapezoidProfile.Constraints(Math.PI/4, Math.PI/3);
  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrent = new TrapezoidProfile.State();

  private final TrapezoidProfile.Constraints extenderConstraints = new TrapezoidProfile.Constraints(1, 0.7);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private final ElevatorFeedforward elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 0.8);
  private final SimpleMotorFeedforward pivotFFModel = new SimpleMotorFeedforward(0.02, 0.3);

  public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO extender) {
    this.pivot = pivot;
    this.extender = extender;

    pivotkP.initDefault(0.2);
    extenderkP.initDefault(15);

    this.pivot.configurePID(pivotkP.get(), 0, 0);
    this.extender.configurePID(extenderkP.get(), 0, 0);
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
    pivot.setPositionSetpoint(position, pivotFFModel.calculate(velocity));
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

    extenderCurrent = extenderProfile.calculate(0.02, extenderCurrent, extenderGoal);
    pivotCurrent = pivotProfile.calculate(0.02, pivotCurrent, pivotGoal);

    
    setPositionPivot(pivotCurrent.position, pivotCurrent.velocity);
    
    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("pivot motor", pInputs);
    Logger.processInputs("extender motor", eInputs);

    if (extenderkP.hasChanged(hashCode())) {
      extender.configurePID(extenderkP.get(), 0, 0);
    }

    if (pivotkP.hasChanged(hashCode())) {
      pivot.configurePID(pivotkP.get(), 0, 0);
    }
  }
}
