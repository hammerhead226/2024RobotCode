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
import frc.robot.Constants;

public class ElevatorPivotIOTalonFX implements ElevatorPivotIO {
  private final TalonFX falcon;
  private final CANcoder cancoder;

  private double positionSetpoint;

  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public ElevatorPivotIOTalonFX(int talonID, int cancoderID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit =
        Constants.ElevatorConstants.PIVOT_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ElevatorConstants.PIVOT_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    falcon = new TalonFX(talonID);

    falcon.getConfigurator().apply(config);

    cancoder = new CANcoder(cancoderID);
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    pivotPosition = cancoder.getAbsolutePosition();
    pivotVelocity = falcon.getVelocity();
    appliedVolts = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    // TODO:: make this a constant
    positionSetpoint = 0;

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pivotPosition, pivotVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorPivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, appliedVolts, currentAmps);
    inputs.pivotPosition = Units.rotationsToDegrees(pivotPosition.getValueAsDouble());
    inputs.pivotVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(pivotVelocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    this.positionSetpoint = position;
    falcon.setControl(new PositionVoltage(position, 0, false, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    this.positionSetpoint = pivotPosition.getValueAsDouble();
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
