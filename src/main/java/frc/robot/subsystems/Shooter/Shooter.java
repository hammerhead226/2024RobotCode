// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

@AutoLog
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final FlywheelIO shooterMotor1;

  private final FlywheelIO shooterMotor2;

  private final FeederIO feeder;

  private final FlywheelIOInputsAutoLogged s1Inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged s2Inputs = new FlywheelIOInputsAutoLogged();

  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  private final SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0, 0.03);
  
  public Shooter(FlywheelIO shooterMotor1, FlywheelIO shooterMotor2, FeederIO feeder) {
    this.shooterMotor1 = shooterMotor1;
    this.shooterMotor2 = shooterMotor2;

    this.feeder = feeder;
  }

  public void stopShooterMotors() {
    shooterMotor1.stop();
    shooterMotor2.stop();
  }

  public void stopFeeders() {
    feeder.stop();
  }

  public void runFeeders(double velocity) {
    feeder.setVelocity(velocity);
  }

  public void setShooterVelocitys(double velocity1, double velocity2) {
    shooterMotor1.setVelocity(velocity1, ffModel.calculate(velocity1));
    shooterMotor2.setVelocity(velocity2, ffModel.calculate(velocity2));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    shooterMotor1.updateInputs(s1Inputs);
    shooterMotor2.updateInputs(s2Inputs);

    feeder.updateInputs(feedInputs);

    Logger.processInputs("shooter motor 1", s1Inputs);
    Logger.processInputs("shooter motor 2", s2Inputs);

    Logger.processInputs("feeder motor", feedInputs);
    
  }
}
