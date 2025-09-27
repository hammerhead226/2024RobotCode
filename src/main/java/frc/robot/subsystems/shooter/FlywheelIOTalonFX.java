package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX left;
  private final TalonFX right;

  private final StatusSignal<AngularVelocity> leftVelocityRPS;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftCurrentAmps;
  private final StatusSignal<Angle> leftRotations;
  private double leftSetpointRPM = 0.0;

  private final StatusSignal<AngularVelocity> rightVelocityRPS;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrentAmps;
  private final StatusSignal<Angle> rightRotations;
  private double rightSetpointRPM = 0.0;

  public FlywheelIOTalonFX(int leftID, int rightID) {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.StatorCurrentLimit =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    left = new TalonFX(leftID, Constants.CANBUS);
    right = new TalonFX(rightID, Constants.CANBUS);

    left.getConfigurator().apply(leftConfig);
    right.getConfigurator().apply(rightConfig);

    leftVelocityRPS = left.getVelocity();
    leftRotations = left.getPosition();
    leftAppliedVolts = left.getMotorVoltage();
    leftCurrentAmps = left.getStatorCurrent();

    rightVelocityRPS = right.getVelocity();
    rightRotations = right.getPosition();
    rightAppliedVolts = right.getMotorVoltage();
    rightCurrentAmps = right.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        leftVelocityRPS,
        leftAppliedVolts,
        leftCurrentAmps,
        rightVelocityRPS,
        rightAppliedVolts,
        rightCurrentAmps,
        leftRotations,
        rightRotations);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftVelocityRPS,
        leftAppliedVolts,
        leftCurrentAmps,
        rightVelocityRPS,
        rightAppliedVolts,
        rightCurrentAmps,
        leftRotations,
        rightRotations);

    inputs.leftRotations = leftRotations.getValueAsDouble();
    inputs.rightRotations = rightRotations.getValueAsDouble();

    inputs.leftVelocityRPM = leftVelocityRPS.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.leftVelocitySetpointRPM = leftSetpointRPM;

    inputs.rightVelocityRPM = rightVelocityRPS.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.rightVelocitySetpointRPM = rightSetpointRPM;
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);
  }

  @Override
  public void setVelocityRPS(
      double leftVelocityRPS, double rightVelocityRPS, double leftFFVolts, double rightFFVolts) {
    this.leftSetpointRPM = leftVelocityRPS * 60.;
    this.rightSetpointRPM = rightVelocityRPS * 60.;

    left.setControl(
        new VelocityVoltage(leftVelocityRPS, 0, false, leftFFVolts, 0, false, false, false));
    right.setControl(
        new VelocityVoltage(rightVelocityRPS, 0, false, rightFFVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    leftSetpointRPM = 0;
    rightSetpointRPM = 0;

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
