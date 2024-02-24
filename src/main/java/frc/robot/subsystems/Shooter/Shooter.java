// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final FlywheelIO flywheels;

  private final FeederIO feeder;
  private final DistanceSensorIO dist;

  private final FlywheelIOInputsAutoLogged fInputs = new FlywheelIOInputsAutoLogged();
  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();
  private final DistanceSensorIOInputsAutoLogged sInputs = new DistanceSensorIOInputsAutoLogged();

  private final SimpleMotorFeedforward leftFlyFFModel;
  private final SimpleMotorFeedforward rightFlyFFModel;
  private final SimpleMotorFeedforward feederFFModel;

  private final SysIdRoutine feedSysId;
  private final SysIdRoutine flywheelSysId;

  private static final LoggedTunableNumber flywheelkP = new LoggedTunableNumber("flywheelkP");
  private static final LoggedTunableNumber flywheelkI = new LoggedTunableNumber("flywheelkI");
  private static final LoggedTunableNumber flywheelkD = new LoggedTunableNumber("flywheelkD");

  private static final LoggedTunableNumber feederkP = new LoggedTunableNumber("feederkP");
  private static final LoggedTunableNumber feederkI = new LoggedTunableNumber("feederkI");
  private static final LoggedTunableNumber feederkD = new LoggedTunableNumber("feederkD");

  public Shooter(FlywheelIO flywheels, FeederIO feeder, DistanceSensorIO dist) {
    switch (Constants.currentMode) {
      case REAL:
        // flywheelFFModel = new SimpleMotorFeedforward(0.18836, 7.2613, 0.16957);
        leftFlyFFModel = new SimpleMotorFeedforward(0.18, 0.12, 0); // make constant
        // leftFlyFFModel = new SimpleMotorFeedforward(0, 0, 0);
        rightFlyFFModel = new SimpleMotorFeedforward(0, 0, 0);
        // feederFFModel = new SimpleMotorFeedforward(0.0608, 6.5977, 0.16859);
        feederFFModel = new SimpleMotorFeedforward(0, 0, 0);
        flywheelkP.initDefault(0.4); // make constant
        flywheelkI.initDefault(0);
        flywheelkD.initDefault(0);
        feederkP.initDefault(0.23); // make constant
        feederkI.initDefault(5); // make constant
        feederkD.initDefault(0);
        break;
      case REPLAY:
        leftFlyFFModel = new SimpleMotorFeedforward(0, 0.03);
        rightFlyFFModel = new SimpleMotorFeedforward(0, 0, 0);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      case SIM:
        leftFlyFFModel = new SimpleMotorFeedforward(0, 0.03);
        rightFlyFFModel = new SimpleMotorFeedforward(0, 0, 0);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      default:
        leftFlyFFModel = new SimpleMotorFeedforward(0, 0.03);
        rightFlyFFModel = new SimpleMotorFeedforward(0, 0, 0);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
    }
    this.flywheels = flywheels;
    // TODO:: Make these constants
    flywheels.configurePID(flywheelkP.get(), flywheelkI.get(), flywheelkD.get());

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
    feeder.configurePID(feederkP.get(), feederkI.get(), feederkD.get());

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

    this.dist = dist;
  }

  public void stopShooterMotors() {
    flywheels.stop();
  }

  public void stopFeeders() {
    feeder.stop();
  }

  public void setFeedersRPM(double velocityRPM) {
    feeder.setVelocityRPS(velocityRPM / 60.0, feederFFModel.calculate(velocityRPM / 60.));
  }

  public void setFlywheelRPMs(double velocity1RPM, double velocity2RPM) {
    flywheels.setVelocityRPS(
        velocity1RPM / 60.,
        velocity2RPM / 60.,
        leftFlyFFModel.calculate(velocity1RPM / 60.),
        rightFlyFFModel.calculate(velocity2RPM / 60.));
  }

  public double[] getFlywheelVelocities() {
    return new double[] {fInputs.leftVelocityRPM, fInputs.leftVelocityRPM};
  }

  public double[] getFlywheelErrors() {
    return new double[] {
      fInputs.leftVelocitySetpointRPM - getFlywheelVelocities()[0],
      fInputs.rightVelocitySetpointRPM - getFlywheelVelocities()[1]
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
    dist.updateInputs(sInputs);

    Logger.processInputs("Flywheels", fInputs);
    Logger.processInputs("Feeder", feedInputs);
    Logger.processInputs("Distance Sensor", sInputs);

    if (feederkP.hasChanged(hashCode())
        || feederkD.hasChanged(hashCode())
        || feederkI.hasChanged(hashCode())) {
      feeder.configurePID(feederkP.get(), feederkI.get(), feederkD.get());
    }

    if (flywheelkP.hasChanged(hashCode())
        || flywheelkI.hasChanged(hashCode())
        || flywheelkD.hasChanged(hashCode())) {
      flywheels.configurePID(flywheelkP.get(), flywheelkI.get(), flywheelkD.get());
    }
  }
}
