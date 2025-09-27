package frc.robot.subsystems.shooter;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import frc.robot.Constants;


public class FeederIOSparkMax implements FeederIO {
   SparkMax neo;
   SparkClosedLoopController pid;
  SparkMaxConfig config = new SparkMaxConfig();

  

  private double velocitySetpointRPS = 0;

  public FeederIOSparkMax(int id) {

    neo = new SparkMax(id, MotorType.kBrushless);
    pid = neo.getClosedLoopController();
    config = new SparkMaxConfig();
    

    

    neo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      

    
    neo.setSmartCurrentLimit((int) Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
    neo.setCANTimeout(250);
    neo.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {

    inputs.feederVelocityRPM = neo.getEncoder().getVelocity();
    inputs.velocitySetpointRPM = velocitySetpointRPS * 60.;
    inputs.feederRotations = neo.getEncoder().getPosition();
    inputs.appliedVolts = neo.getAppliedOutput();
    inputs.currentAmps = neo.getOutputCurrent();
  }

  @Override
  public void setVelocityRPS(double velocityRPS, double ffVolts) {
    this.velocitySetpointRPS = velocityRPS;
    pid.setReference(velocityRPS, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
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
