package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX left;
  private final TalonFX right;

  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> leftCurrentAmps;
  private double leftSetpoint = 0.0;

  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> rightCurrentAmps;
  private double rightSetpoint = 0.0;

  public FlywheelIOTalonFX(int leftID, int rightID) {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    left = new TalonFX(leftID, Constants.CANBUS);
    right = new TalonFX(rightID, Constants.CANBUS);

    left.getConfigurator().apply(leftConfig);
    right.getConfigurator().apply(rightConfig);

    leftVelocity = left.getVelocity();
    leftAppliedVolts = left.getMotorVoltage();
    leftCurrentAmps = left.getStatorCurrent();

    rightVelocity = right.getVelocity();
    rightAppliedVolts = right.getMotorVoltage();
    rightCurrentAmps = right.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, leftVelocity, leftAppliedVolts, leftCurrentAmps, rightVelocity, rightAppliedVolts, rightCurrentAmps);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftVelocity, leftAppliedVolts, leftCurrentAmps, rightVelocity, rightAppliedVolts, rightCurrentAmps);

    inputs.leftVelocityRPM = leftVelocity.getValueAsDouble();
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftVelocitySetpoint = leftSetpoint;

    inputs.rightVelocityRPM = rightVelocity.getValueAsDouble();
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightVelocitySetpoint = rightSetpoint;
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);
  }

  @Override
  public void setVelocityRPM(double leftVelocity, double rightVelocity, double ffVolts) {
    this.leftSetpoint = leftVelocity;
    this.rightSetpoint = rightVelocity;

    left.setControl(
        new VelocityVoltage(leftVelocity, 0, false, ffVolts, 0, false, false, false));
    
    left.setControl(
        new VelocityVoltage(rightVelocity, 0, false, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    leftSetpoint = 0;
    rightSetpoint = 0;

    left.stopMotor();
    right.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs configs = new Slot0Configs();

    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;

    left.getConfigurator().apply(configs);
    right.getConfigurator().apply(configs);
  }
}
