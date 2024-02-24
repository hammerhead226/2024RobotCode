// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final FlywheelIO flywheels;
  private final FeederIO feeder;

  private final FlywheelIOInputsAutoLogged fInputs = new FlywheelIOInputsAutoLogged();
  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  private final SimpleMotorFeedforward flywheelFFModel;
  private final SimpleMotorFeedforward feederFFModel;

  private final SysIdRoutine feedSysId;
  private final SysIdRoutine flywheelSysId;

  private static final LoggedTunableNumber feederkP = new LoggedTunableNumber("feederkP");
  private static final LoggedTunableNumber flywheelkP = new LoggedTunableNumber("flywheelkP");

  public Shooter(FlywheelIO flywheels, FeederIO feeder) {
    switch (Constants.currentMode) {
      case REAL:
        flywheelFFModel = new SimpleMotorFeedforward(0, 3);
        feederFFModel = new SimpleMotorFeedforward(0, 0.3);

        flywheelkP.initDefault(0);
        feederkP.initDefault(0);
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
    // TODO:: Make these constants
    flywheels.configurePID(flywheelkP.get(), 0, 0);

    // Configure SysId
    flywheelSysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Flywheels/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              flywheels.setVoltage(voltage.in(Volts));
            },
            null,
            this));


    this.feeder = feeder;
    // TODO:: Make these constants
    feeder.configurePID(feederkP.get(), 0, 0);

    // Configure SysId
    feedSysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Feeder/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              feeder.runCharacterization(voltage.in(Volts));
            },
            null,
            this));

  }

  public void stopShooterMotors() {
    flywheels.stop();
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

    /** Returns a command to run a quasistatic test in the specified direction. */
  public Command feedSysIDQuasistic(SysIdRoutine.Direction direction) {
    return feedSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command feedSysIDDynamiCommand(SysIdRoutine.Direction direction) {
    return feedSysId.dynamic(direction);
  }

    /** Returns a command to run a quasistatic test in the specified direction. */
  public Command flywheelSysIDQuasistic(SysIdRoutine.Direction direction) {
    return flywheelSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command flywheelSysIDDynamiCommand(SysIdRoutine.Direction direction) {
    return flywheelSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    flywheels.updateInputs(fInputs);
    feeder.updateInputs(feedInputs);

    Logger.processInputs("Flywheels", fInputs);
    Logger.processInputs("Feeder", feedInputs);

    if (feederkP.hasChanged(hashCode())) {
      feeder.configurePID(feederkP.get(), 0, 0);
    }

    if (flywheelkP.hasChanged(hashCode())) {
      flywheels.configurePID(flywheelkP.get(), 0, 0);
    }
  }
}
