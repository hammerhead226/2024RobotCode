package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ElevatorPivotIOTalonFX implements ElevatorPivotIO {
  private final TalonFX falcon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> pivotAbsolutePosition;
  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public ElevatorPivotIOTalonFX(int talonID, int cancoderID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    falcon = new TalonFX(talonID);

    falcon.getConfigurator().apply(config);

    cancoder = new CANcoder(cancoderID);
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    pivotAbsolutePosition = cancoder.getAbsolutePosition();
    pivotPosition = falcon.getPosition();
    pivotVelocity = falcon.getVelocity();
    appliedVolts = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pivotAbsolutePosition, pivotPosition, pivotVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorPivotIOInputs inputs) {
    inputs.pivotAbsolutePosition = Units.rotationsToDegrees(pivotAbsolutePosition.getValueAsDouble());
    inputs.pivotPosition = Units.rotationsToDegrees(pivotPosition.getValueAsDouble());
    inputs.pivotVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(pivotVelocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    falcon.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void setPosition(double position) {
    falcon.setControl(new PositionVoltage(position));
  }

  @Override
  public void stop() {
    falcon.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    falcon.getConfigurator().apply(config);
  }
}
