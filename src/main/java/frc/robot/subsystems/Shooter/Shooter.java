// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final FlywheelIO flywheels;

  // private final FlywheelIO shooterMotor2;

  private final FeederIO feeder;

  private final FlywheelIOInputsAutoLogged fInputs = new FlywheelIOInputsAutoLogged();
  // private final FlywheelIOInputsAutoLogged s2Inputs = new FlywheelIOInputsAutoLogged();

  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  private final SimpleMotorFeedforward flywheelFFModel;
  private final SimpleMotorFeedforward feederFFModel;

  public Shooter(FlywheelIO flywheels, FeederIO feeder) {
    switch (Constants.currentMode) {
      case REAL:
        flywheelFFModel = new SimpleMotorFeedforward(0, 3);
        feederFFModel = new SimpleMotorFeedforward(0, 0.3);
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
    this.flywheels = flywheels;
    // this.shooterMotor2 = shooterMotor2;
    // TODO:: Make these constants
    flywheels.configurePID(0.5, 0, 0);
    // shooterMotor2.configurePID(0.5, 0, 0);

    this.feeder = feeder;
    // TODO:: Make these constants
    feeder.configurePID(0.5, 0, 0);
  }

  public void stopShooterMotors() {
    flywheels.stop();
    // shooterMotor2.stop();
  }

  public void stopFeeders() {
    feeder.stop();
  }

  public void runFeeders(double velocity) {
    feeder.setVelocityRPM(velocity, feederFFModel.calculate(velocity));
  }

  public void setFlywheelRPMs(double velocity1, double velocity2) {
    flywheels.setVelocityRPM(velocity1, velocity2, flywheelFFModel.calculate(velocity1));
  }

  public double[] getFlywheelVelocities() {
    return new double[] {fInputs.leftVelocityRPM, fInputs.rightVelocityRPM};
  }

  public double[] getFlywheelErrors() {
    return new double[] {
      fInputs.leftVelocitySetpoint - getFlywheelVelocities()[0],
      fInputs.rightVelocitySetpoint - getFlywheelVelocities()[1]
    };
  }

  public boolean atFlywheelSetpoints() {
    return (Math.abs(getFlywheelErrors()[0]) <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD
        && getFlywheelErrors()[1] <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    flywheels.updateInputs(fInputs);
    feeder.updateInputs(feedInputs);

    Logger.processInputs("Flywheels", fInputs);
    Logger.processInputs("Feeder", feedInputs);
  }
}
