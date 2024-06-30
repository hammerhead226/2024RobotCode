package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.mt2VisionPose =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).pose;
    inputs.mt1VisionPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LL_ALIGN);
    inputs.timestampSeconds =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).timestampSeconds;
    inputs.tagCount =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).tagCount;
    inputs.tagSpan =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).tagSpan;
    inputs.latency =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).latency;
    inputs.avgTagDist =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).avgTagDist;
    inputs.avgTagArea =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN).avgTagArea;

    // Raw Limelight Data
    inputs.iTX = LimelightHelpers.getTX(Constants.LL_INTAKE);
    inputs.iTY = LimelightHelpers.getTY(Constants.LL_INTAKE);
    inputs.iTA = LimelightHelpers.getTA(Constants.LL_INTAKE);
    inputs.iHB =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "hb").getDouble(0.0);
    inputs.iTV = LimelightHelpers.getTV(Constants.LL_INTAKE);
    inputs.iPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(Constants.LL_INTAKE);
    inputs.iCAPTURELATENCY = LimelightHelpers.getLatency_Capture(Constants.LL_INTAKE);
    inputs.iTHOR =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "thor").getDouble(0.0);
    inputs.iTVERT =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "tvert").getDouble(0.0);

    inputs.aTX = LimelightHelpers.getTX(Constants.LL_ALIGN);
    inputs.aTY = LimelightHelpers.getTY(Constants.LL_ALIGN);
    inputs.aTA = LimelightHelpers.getTA(Constants.LL_ALIGN);
    inputs.aHB = LimelightHelpers.getLimelightNTTableEntry(Constants.LL_ALIGN, "hb").getDouble(0.0);
    inputs.aTV = LimelightHelpers.getTV(Constants.LL_ALIGN);
    inputs.aPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(Constants.LL_ALIGN);
    inputs.aCAPTURELATENCY = LimelightHelpers.getLatency_Capture(Constants.LL_ALIGN);
    inputs.aTHOR =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_ALIGN, "thor").getDouble(0.0);
    inputs.aTVERT =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_ALIGN, "tvert").getDouble(0.0);
  }
}
