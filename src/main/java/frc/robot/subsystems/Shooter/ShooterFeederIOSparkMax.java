package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class ShooterFeederIOSparkMax implements ShooterFeederIO {
  private final CANSparkMax neo;
  private final SparkPIDController pid;

  public ShooterFeederIOSparkMax(int id) {

    neo = new CANSparkMax(id, MotorType.kBrushless);
    pid = neo.getPIDController();

    neo.restoreFactoryDefaults();
    neo.setSmartCurrentLimit(Constants.shooterCurrentLimits.FEEDER_SPARK_MAX_CURRENT_LIMIT);
    neo.setCANTimeout(250);
    neo.burnFlash();
  }

  @Override
  public void updateInputs(ShooterFeederIOInputs inputs) {
    inputs.feederVelocity = neo.getEncoder().getVelocity();

    inputs.appliedVolts = neo.getBusVoltage();
    inputs.currentAmps = neo.getOutputCurrent();
  }

  @Override
  public void setVelocity(double velocity) {
    pid.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    neo.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
