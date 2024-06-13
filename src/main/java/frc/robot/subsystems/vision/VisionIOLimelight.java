package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    DriverStation.Alliance alliance;

    int lastHBIntake = -1;
    int lastHBAlign = -1;

    public VisionIOLimelight(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (alliance.equals(DriverStation.Alliance.Blue)) inputs.visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN);
        else inputs.visionPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.LL_ALIGN);

        inputs.robotRelNotePoseCorrected = new TimestampedT2d(getRobotRelativeNoteTranslation2d()[0], Timer.getFPGATimestamp());
        inputs.robotRelNotePoseRaw = getRobotRelativeNoteTranslation2d()[1];

        inputs.aprilTagLimelightConnected = isLimelightOn(Constants.LL_ALIGN);
        inputs.intakeLimelightConnected = isLimelightOn(Constants.LL_INTAKE);
    }

    private boolean isLimelightOn(String ll) {
        if (ll.equals(Constants.LL_ALIGN)) {
            return LimelightHelpers.getLimelightNTTableEntry(ll, "hb").getDouble(0.0) != lastHBAlign;
        } else {
            return LimelightHelpers.getLimelightNTTableEntry(ll, "hb").getDouble(0.0) != lastHBIntake;
        }
    }

    public Translation2d[] getRobotRelativeNoteTranslation2d() {

        double distInch = (1 / (40 - ((30) * LimelightHelpers.getTY(Constants.LL_INTAKE) / 23)) * 1000); // Convert degrees to inch
        double noteYawAngleDegCorrected =
            -LimelightHelpers.getTX(Constants.LL_INTAKE) - 4; // account for static offset, reverse to be CCW+
        double radiusInchCorrected =
            distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegCorrected));

        double noteYawAngleDegRaw = -LimelightHelpers.getTX(Constants.LL_INTAKE); // account for static offset, reverse to be CCW+
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

        return new Translation2d[] {roboRelNoteLocT2dCorrected, roboRelNoteLocT2dRaw};
    }
}
