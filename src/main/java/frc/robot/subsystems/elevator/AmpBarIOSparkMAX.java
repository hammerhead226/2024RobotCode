package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class AmpBarIOSparkMAX implements AmpBarIO {

  private final CANSparkMax barMotor;
  private final SparkPIDController pid;
  private double barPositionSetpoint;

  public AmpBarIOSparkMAX(int motorID) {

    barMotor = new CANSparkMax(motorID, CANSparkLowLevel.MotorType.kBrushless);

    pid = barMotor.getPIDController();
    pid.setFeedbackDevice(barMotor.getAbsoluteEncoder());

    barMotor.restoreFactoryDefaults();
    barMotor.setCANTimeout(250);
    barMotor.setSmartCurrentLimit(20);
    barMotor.burnFlash();
    barMotor.clearFaults();

    barPositionSetpoint = 0;
  }

  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    inputs.barVelocityDegsPerSec = (barMotor.getEncoder().getVelocity() / 15) * 6;
    inputs.barPositionDegrees = (barMotor.getEncoder().getPosition() / 15) * 360;
    inputs.barPositionSetpointDegrees = barPositionSetpoint;
    inputs.currentAmps = barMotor.getOutputCurrent();
    inputs.appliedVolts = barMotor.getAppliedOutput();
  }

  @Override
  public void setBrakeMode(boolean bool) {

    if (bool) {
      barMotor.setIdleMode(IdleMode.kBrake);
    } else {
      barMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void setPositionSetpoint(double positionDegs, double ffVolts) {

    this.barPositionSetpoint = positionDegs;
    pid.setReference(
        (positionDegs / 360) * 15, ControlType.kPosition, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    barMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    barMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }
}
