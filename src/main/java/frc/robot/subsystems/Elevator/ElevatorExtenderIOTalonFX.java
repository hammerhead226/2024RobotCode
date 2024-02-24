package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.Conversions;

public class ElevatorExtenderIOTalonFX implements ElevatorExtenderIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private double positionSetpoint;
  private final StatusSignal<Double> elevatorPosition;
  private final StatusSignal<Double> elevatorVelocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public ElevatorExtenderIOTalonFX(int lead, int follow) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ElevatorConstants.EXTENDER_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ElevatorConstants.EXTENDER_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leader = new TalonFX(lead, Constants.CANBUS);
    follower = new TalonFX(follow, Constants.CANBUS);

    leader.getConfigurator().apply(config);

    positionSetpoint = Constants.ElevatorConstants.EXTENDER_RETRACT;

    follower.setControl(new Follower(lead, true));

    elevatorPosition = leader.getPosition();
    elevatorVelocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorExtenderIOInputs inputs) {
    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
    inputs.elevatorPosition =
        Conversions.motorRotToInches(elevatorPosition.getValueAsDouble(), 5.97, 15);
    inputs.elevatorVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(elevatorVelocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    this.positionSetpoint = position;
    leader.setControl(
        new PositionVoltage(
            Conversions.inchesToMotorRot(position, 5.97, 15),
            0,
            false,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    this.positionSetpoint = elevatorPosition.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leader.getConfigurator().apply(config);
  }
}
