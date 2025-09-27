// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Gear ratios for SDS MK4i L3, adjust as necessary
  private final double DRIVE_GEAR_RATIO = Constants.ModuleConstants.DRIVE_GEAR_RATIO;
  private final double TURN_GEAR_RATIO = Constants.ModuleConstants.TURN_GEAR_RATIO;

  private final boolean isTurnMotorInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(0, Constants.CANBUS);
        turnTalon = new TalonFX(4, Constants.CANBUS);
        cancoder = new CANcoder(0, Constants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(-2.14); // MUST BE CALIBRATED
        break;
      case 1:
        driveTalon = new TalonFX(1, Constants.CANBUS);
        turnTalon = new TalonFX(5, Constants.CANBUS);
        cancoder = new CANcoder(1, Constants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(-1.82); // MUST BE CALIBRATED
        break;
      case 2:
        driveTalon = new TalonFX(2, Constants.CANBUS);
        turnTalon = new TalonFX(6, Constants.CANBUS);
        cancoder = new CANcoder(2, Constants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(1.74); // MUST BE CALIBRATED
        break;
      case 3:
        driveTalon = new TalonFX(3, Constants.CANBUS);
        turnTalon = new TalonFX(7, Constants.CANBUS);
        cancoder = new CANcoder(3, Constants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(0.98); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit =
        Constants.ModuleConstants.DRIVE_STATOR_CURRENT_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ModuleConstants.DRIVE_STATOR_CURRENT_LIMIT_ENABLED;
    driveConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ModuleConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.ModuleConstants.DRIVE_SUPPLY_CURRENT_LIMIT_ENABLED;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);
    // setDriveOpenLoopRamp(Constants.SwerveConstants.OPEN_LOOP_RAMP_SEC);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit =
        Constants.ModuleConstants.TURN_STATOR_CURRENT_LIMIT;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ModuleConstants.TURN_STATOR_CURRENT_LIMIT_ENABLED;
    turnConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ModuleConstants.TURN_SUPPLY_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.ModuleConstants.TURN_SUPPLY_CURRENT_LIMIT_ENABLED;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition, turnPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveOpenLoopRamp(double seconds) {
    var ramping = new OpenLoopRampsConfigs();

    ramping.VoltageOpenLoopRampPeriod = seconds;
    // driveTalon.getConfigurator().apply(ramping);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
