// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final FlywheelIO shooterMotor1;

  private final FlywheelIO shooterMotor2;

  private final FeederIO feeder;

  private final FlywheelIOInputsAutoLogged s1Inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged s2Inputs = new FlywheelIOInputsAutoLogged();

  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  private final SimpleMotorFeedforward flywheelFFModel;
  private final SimpleMotorFeedforward feederFFModel;

  public Shooter(FlywheelIO shooterMotor1, FlywheelIO shooterMotor2, FeederIO feeder) {
    switch (Constants.currentMode) {
      case REAL:
        flywheelFFModel = new SimpleMotorFeedforward(0, 0.03);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      case REPLAY:
        flywheelFFModel = new SimpleMotorFeedforward(0, 0.03);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      case SIM:
        flywheelFFModel = new SimpleMotorFeedforward(0, 0.03);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      default:
        flywheelFFModel = new SimpleMotorFeedforward(0, 0.03);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
    }
    this.shooterMotor1 = shooterMotor1;
    this.shooterMotor2 = shooterMotor2;
    // TODO:: Make these constants
    shooterMotor1.configurePID(0, 0, 0);
    shooterMotor2.configurePID(0, 0, 0);

    this.feeder = feeder;
    // TODO:: Make these constants
    feeder.configurePID(0, 0, 0);
  }

  public void stopShooterMotors() {
    shooterMotor1.stop();
    shooterMotor2.stop();
  }

  public void stopFeeders() {
    feeder.stop();
  }

  public void runFeeders(double velocity) {
    feeder.setVelocity(velocity, feederFFModel.calculate(velocity));
  }

  public void setShooterVelocitys(double velocity1, double velocity2) {
    shooterMotor1.setVelocity(velocity1, flywheelFFModel.calculate(velocity1));
    shooterMotor2.setVelocity(velocity2, flywheelFFModel.calculate(velocity2));
  }

  public double[] getFlywheelVelocities() {
    return new double[] {s1Inputs.shooterVelocity, s2Inputs.shooterVelocity};
  }

  public double[] getFlywheelErrors() {
    return new double[] {s1Inputs.velocitySetpoint - getFlywheelVelocities()[0], s2Inputs.velocitySetpoint - getFlywheelVelocities()[1]};
  }

  public boolean atFlywheelSetpoints() {
    return (Math.abs(getFlywheelErrors()[0]) <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD && getFlywheelErrors()[1] <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    shooterMotor1.updateInputs(s1Inputs);
    shooterMotor2.updateInputs(s2Inputs);

    feeder.updateInputs(feedInputs);

    Logger.processInputs("Flywheel 1", s1Inputs);
    Logger.processInputs("Flywheel 2", s2Inputs);

    Logger.processInputs("Feeder", feedInputs);
  }
}
