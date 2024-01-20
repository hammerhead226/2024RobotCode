// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

@AutoLog
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIO shooterMotor1;

  private final ShooterIO shooterMotor2;

  private final ShooterFeederIO feederMotor1;

  private final ShooterFeederIO feederMotor2;

  private final ShooterIOInputsAutoLogged s1Inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOInputsAutoLogged s2Inputs = new ShooterIOInputsAutoLogged();

  private final ShooterFeederIOInputsAutoLogged feed1Inputs = new ShooterFeederIOInputsAutoLogged();
  private final ShooterFeederIOInputsAutoLogged feed2Inputs = new ShooterFeederIOInputsAutoLogged();

  public Shooter(
      ShooterIO shooterMotor1,
      ShooterIO shooterMotor2,
      ShooterFeederIO feeder1,
      ShooterFeederIO feeder2) {

    this.shooterMotor1 = shooterMotor1;
    this.shooterMotor2 = shooterMotor2;

    feederMotor1 = feeder1;
    feederMotor2 = feeder2;
  }

  public void stopShooterMotors() {
    shooterMotor1.stop();
    shooterMotor2.stop();
  }

  public void stopFeeders() {
    feederMotor1.stop();
    feederMotor2.stop();
  }

  public void runFeeders(double velocity) {
    feederMotor1.setVelocity(velocity);
    feederMotor2.setVelocity(velocity);
  }

  public void setShooterVelocitys(double velocity1, double velocity2) {

    shooterMotor1.setVelocity(velocity1);
    shooterMotor2.setVelocity(velocity2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    shooterMotor1.updateInputs(s1Inputs);
    shooterMotor2.updateInputs(s2Inputs);

    feederMotor1.updateInputs(feed1Inputs);
    feederMotor2.updateInputs(feed2Inputs);

    Logger.processInputs("shooter motor 1", s1Inputs);
    Logger.processInputs("shooter motor 2", s2Inputs);

    Logger.processInputs("feeder motor 1", feed1Inputs);
    Logger.processInputs("feeder motor 2", feed2Inputs);
  }
}
