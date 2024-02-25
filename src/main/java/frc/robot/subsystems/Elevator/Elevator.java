package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevator;

  private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber extenderkP =
      new LoggedTunableNumber("elevatorExtenderkP");

  private static final LoggedTunableNumber extenderkI =
      new LoggedTunableNumber("elevatorExtenderkI");

  private final TrapezoidProfile extenderProfile;
  private final TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(30, 85);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private final ElevatorFeedforward elevatorFFModel;

  private final SysIdRoutine sysId;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;

    switch (Constants.currentMode) {
      case REAL:
        elevatorFFModel = new ElevatorFeedforward(0, 0.13, 0);
        extenderkP.initDefault(0.35);
        extenderkI.initDefault(0);
        break;
      case REPLAY:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        extenderkP.initDefault(15);
        break;
      case SIM:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        extenderkP.initDefault(15);
        break;
      default:
        elevatorFFModel = new ElevatorFeedforward(0.02, 0.05, 1.4);
        extenderkP.initDefault(15);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  elevator.runCharacterization(voltage.in(Volts));
                },
                null,
                this));

    setExtenderGoal(0.162);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    this.elevator.configurePID(extenderkP.get(), 0, 0);
  }

  public double getExtenderPosition() {
    return eInputs.elevatorPosition;
  }

  private double getExtenderError() {
    return eInputs.positionSetpoint - eInputs.elevatorPosition;
  }

  public boolean extenderAtSetpoint() {
    return (Math.abs(getExtenderError()) <= Constants.ElevatorConstants.THRESHOLD);
  }

  public void setExtenderGoal(double setpoint) {
    extenderGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setPositionExtend(double position, double velocity) {
    elevator.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
  }

  public void elevatorStop() {
    elevator.stop();
  }

  public double calculateAngle() {
    double angle = 0.0;
    return angle;
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

    elevator.updateInputs(eInputs);

    extenderCurrent =
        extenderProfile.calculate(Constants.LOOP_PERIOD_SECS, extenderCurrent, extenderGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);

    if (extenderkP.hasChanged(hashCode()) || extenderkI.hasChanged(hashCode())) {
      elevator.configurePID(extenderkP.get(), extenderkI.get(), 0);
    }
  }
}
