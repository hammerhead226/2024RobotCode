package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevator;

  private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(30, 85);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private double goal;

  private final ElevatorFeedforward elevatorFFModel;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;

    switch (Constants.getMode()) {
      case REAL:
        kS.initDefault(0);
        kG.initDefault(0.25);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0.44);
        kI.initDefault(0);
        break;
      case REPLAY:
        kS.initDefault(0);
        kG.initDefault(0.13);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(15);
        kI.initDefault(0);
        break;
      case SIM:
        kS.initDefault(0);
        kG.initDefault(0.04);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(1);
        kI.initDefault(0);
        break;
      default:
        kS.initDefault(0);
        kG.initDefault(0.13);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(15);
        kI.initDefault(0);
        break;
    }

    setExtenderGoal(0.162);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    this.elevator.configurePID(kP.get(), 0, 0);
    elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  public boolean atGoal() {
    return (Math.abs(eInputs.elevatorPosition - goal) <= Constants.ElevatorConstants.THRESHOLD);
  }

  public double getElevatorPosition() {
    return eInputs.elevatorPosition;
  }

  private double getElevatorError() {
    return eInputs.positionSetpoint - eInputs.elevatorPosition;
  }

  public boolean elevatorAtSetpoint() {
    return (Math.abs(getElevatorError()) <= Constants.ElevatorConstants.THRESHOLD);
  }

  public void setExtenderGoal(double setpoint) {
    goal = setpoint;
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

  public void setConstraints(
      double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
    extenderConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
  }

  public boolean isExtended() {
    return extenderGoal.position == Constants.ElevatorConstants.EXTEND_SETPOINT_INCH;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

    elevator.updateInputs(eInputs);

    extenderCurrent =
        extenderProfile.calculate(Constants.LOOP_PERIOD_SECS, extenderCurrent, extenderGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      elevator.configurePID(kP.get(), kI.get(), 0);
    }
  }
}
