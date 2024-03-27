// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  VisionIO io;
  VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public Translation2d getRobotRelNoteLocation(double gyro) {
    double distance = inputs.distanceMeters;
    double angle = inputs.angleDegs;

    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      // if (gyro )
    }

    return new Translation2d();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Vision", inputs);
    // This method will be called once per scheduler run
  }
}
