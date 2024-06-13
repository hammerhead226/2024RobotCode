package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    DriverStation.Alliance alliance;

    int lastHBIntake = -1;
    int lastHBAlign = -1;

    final double taThreshold = 0.1;

    public VisionIOLimelight(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (alliance.equals(DriverStation.Alliance.Blue)) inputs.visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN);
        else inputs.visionPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.LL_ALIGN);

        inputs.noteData.yawDegs = -LimelightHelpers.getTX(Constants.LL_INTAKE) - 4;
        inputs.noteData.distanceInches = inputs.noteData.yawDegs / Math.cos(Units.degreesToRadians((1. / (40 - ((30) * LimelightHelpers.getTY(Constants.LL_INTAKE) / 23.)) * 1000)));
        if (LimelightHelpers.getTA(Constants.LL_INTAKE) >= taThreshold) {
            inputs.noteData.timestamp = Timer.getFPGATimestamp(); // check if this is in seconds or milliseconds
        }

        inputs.currentTime = Timer.getFPGATimestamp();


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
}
