// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;


public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final RobotContainer r = new RobotContainer();
  private final VisionIO io;
  private final VisionIOInputsAutoLogged vInputs = new VisionIOInputsAutoLogged();

  private Translation2d lastNoteLocation;


  public class TimestampedPose2d{
      Pose2d pose;
      double time;
  }

  CircularBuffer<TimestampedPose2d> robotPoseBuffer;

  public Vision(VisionIO io) {
    this.io = io;

    robotPoseBuffer = new CircularBuffer<>(11);
  }

  private void updatePoseBuffer() {
    TimestampedPose2d now = new TimestampedPose2d();
    now.pose = r.getDrive().getPose();
    now.time = Timer.getFPGATimestamp();
    robotPoseBuffer.addFirst(now);
  }

  private Pose2d posePicker(double time) {
    TimestampedPose2d prev = robotPoseBuffer.getFirst();
    for (int i = 0; i < robotPoseBuffer.size(); i++) {
      TimestampedPose2d next = robotPoseBuffer.get(i);
      double delta = next.time - time;
      if (delta < 0) {
        double t = ((time - next.time) / (prev.time - next.time));
        return next.pose.interpolate(prev.pose, t);
      }
    }
    // if the time is before everything in the buffer return the oldest thing
    return robotPoseBuffer.getLast().pose;
  }

  public Translation2d calculateNotePositionFieldRelative() {

    double distInch = (1 / (40 - ((30) * getIntakeLLTy() / 23)) * 1000); // Convert degrees to inch
    double noteYawAngleDegCorrected =
        vInputs.noteData.yawDegs; // account for static offset, reverse to be CCW+
    double radiusInchCorrected =
        vInputs.noteData.distanceInches;

    double noteYawAngleDegRaw = -getIntakeLLTx(); // account for static offset, reverse to be CCW+
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
    Pose2d pickedRobotPose =
        posePicker(
            Timer.getFPGATimestamp()
                - LimelightHelpers.getLatency_Pipeline(Constants.LL_INTAKE)
                - LimelightHelpers.getLatency_Capture(Constants.LL_INTAKE));
    Translation2d fieldRelNoteLocT2dCorrected =
        roboRelNoteLocT2dCorrected
            .rotateBy(pickedRobotPose.getRotation())
            .plus(pickedRobotPose.getTranslation());

    Translation2d fieldRelNoteLocT2dRaw =
        roboRelNoteLocT2dRaw
            .rotateBy(pickedRobotPose.getRotation())
            .plus(pickedRobotPose.getTranslation());

    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dRaw", fieldRelNoteLocT2dRaw);
    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dCorrected", fieldRelNoteLocT2dCorrected);
    Logger.recordOutput(
        "distance from center of robot",
        Units.metersToInches(fieldRelNoteLocT2dCorrected.getDistance(r.getDrive().getPose().getTranslation())));
    return fieldRelNoteLocT2dCorrected;
  }

  public Translation2d getCachedNoteLocation(){
    return lastNoteLocation;
}

  public double getIntakeLLTx() {
     return LimelightHelpers.getTX(Constants.LL_INTAKE);
  }

  public double getIntakeLLTy() {
     return LimelightHelpers.getTY(Constants.LL_INTAKE);
  }

  @Override
  public void periodic() {
    io.updateInputs(vInputs);

    Logger.processInputs("Vision", vInputs);

    updatePoseBuffer();

    // This method will be called once per scheduler run
  }
}
