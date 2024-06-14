// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final RobotContainer r = new RobotContainer();

  private final VisionIO io;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  private Translation2d lastNoteLocation;

  public class TimestampedPose2d {
    Pose2d pose;
    double time;
  }

  CircularBuffer<TimestampedPose2d> robotPoseBuffer;

  public Vision(VisionIO io) {
    this.io = io;

    robotPoseBuffer = new CircularBuffer<>(11);
  }

  public Translation2d calculateNotePositionFieldRelative(Pose2d robotPose) {

    double distInch = (1 / (40 - ((30) * visionInputs.iTY / 23)) * 1000); // Convert degrees to inch
    double noteYawAngleDegCorrected =
        -visionInputs.iTX - 4; // account for static offset, reverse to be CCW+
    double radiusInchCorrected =
        distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegCorrected));

    double noteYawAngleDegRaw = -visionInputs.iTX; // account for static offset, reverse to be CCW+
    double radiusInchRaw = distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegRaw));

    Logger.recordOutput("NoteTracking/distInch", distInch);
    Logger.recordOutput("NoteTracking/noteYawAngleDegCorrected", noteYawAngleDegCorrected);
    Logger.recordOutput("NoteTracking/noteYawAngleDegRaw", noteYawAngleDegRaw);
    Logger.recordOutput("NoteTracking/radiusCorrected", radiusInchCorrected);

    // camera relative -> bot relative -> field relative
    Translation2d camRelNoteLocT2dCorrected =
        new Translation2d(
            Units.inchesToMeters(radiusInchCorrected),
            Rotation2d.fromDegrees(noteYawAngleDegCorrected));
    Logger.recordOutput("NoteTracking/camRelNoteLocT2dCorrected", camRelNoteLocT2dCorrected);

    Translation2d camRelNoteLocT2dRaw =
        new Translation2d(
            Units.inchesToMeters(radiusInchRaw), Rotation2d.fromDegrees(noteYawAngleDegRaw));

    Translation2d roboRelNoteLocT2dRaw =
        camRelNoteLocT2dRaw
            .rotateBy(Rotation2d.fromDegrees(0))
            .plus(new Translation2d(Units.inchesToMeters(12), 0));

    Translation2d roboRelNoteLocT2dCorrected =
        camRelNoteLocT2dCorrected
            .rotateBy(Rotation2d.fromDegrees(0))
            .plus(new Translation2d(Units.inchesToMeters(12), 0));
    Logger.recordOutput("NoteTracking/roboRelNoteLocT2dCorrected", roboRelNoteLocT2dCorrected);

    Translation2d fieldRelNoteLocT2dCorrected =
        roboRelNoteLocT2dCorrected
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());

    Translation2d fieldRelNoteLocT2dRaw =
        roboRelNoteLocT2dRaw.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dRaw", fieldRelNoteLocT2dRaw);
    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dCorrected", fieldRelNoteLocT2dCorrected);
    return fieldRelNoteLocT2dCorrected;
  }

  public Translation2d getCachedNoteLocation() {
    return lastNoteLocation;
  }

  @Override
  public void periodic() {
    io.updateInputs(visionInputs);

    Logger.processInputs("Vision", visionInputs);

    // updatePoseBuffer();

    // This method will be called once per scheduler run
  }
}
