package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

 
import frc.robot.Constants;

public class IntakeRollerIOSparkFlex implements IntakeRollerIO {
  private final SparkMax rollers;

  public IntakeRollerIOSparkFlex(int id) {
    SparkMax rollers = new SparkMax(id, MotorType.kBrushless);
    SparkClosedLoopController pid = rollers.getClosedLoopController();
    
    
    
     rollers.setSmart(5);
    rollers.setCANTimeout(250);
    SparkMaxConfig.
    rollers.burnFlash();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.rollerRotations = rollers.getEncoder().getPosition();
    inputs.rollerVelocityRPM = rollers.getEncoder().getVelocity();

    inputs.appliedVolts = rollers.getAppliedOutput() * 12.;
    inputs.currentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }
}
