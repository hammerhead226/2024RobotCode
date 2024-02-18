package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

  private final TrapezoidProfile.Constraints pivotConstraints = new TrapezoidProfile.Constraints(Math.PI/8, 3 * Math.PI / 2);
  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();

  private final TrapezoidProfile.Constraints extenderConstraints = new TrapezoidProfile.Constraints(1, 0.7);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderSetpoint = new TrapezoidProfile.State();

  private final ElevatorFeedforward ffModel = new ElevatorFeedforward(0.02, 0.05, 0.8);

  public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO extender) {
    this.pivot = pivot;
    this.extender = extender;

    pivotkP.initDefault(0.5);
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

  public void setPositionElevator(double position, double velocity) {
    extender.setPositionSetpoint(position, ffModel.calculate(velocity));
  }

  public void setPositionPivot(double position) {
    pivot.setPosition(position);
  }

  public void setPivotVelocity(double pivotVelocity) {
    pivot.setVelocity(pivotVelocity);
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

    extenderSetpoint = extenderProfile.calculate(0.02, extenderSetpoint, extenderGoal);
    pivotSetpoint = pivotProfile.calculate(0.02, pivotSetpoint, pivotGoal);

    
    pivot.setVelocity(pivotSetpoint.velocity);
    pivot.setPosition(pivotSetpoint.position);
    
    setPositionElevator(extenderSetpoint.position, extenderSetpoint.velocity);

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
