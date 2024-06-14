package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.visionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LL_ALIGN);

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

    inputs.aTX = LimelightHelpers.getTX(Constants.LL_INTAKE);
    inputs.aTY = LimelightHelpers.getTY(Constants.LL_INTAKE);
    inputs.aTA = LimelightHelpers.getTA(Constants.LL_INTAKE);
    inputs.aHB =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "hb").getDouble(0.0);
    inputs.aTV = LimelightHelpers.getTV(Constants.LL_INTAKE);
    inputs.aPIPELINELATENCY = LimelightHelpers.getLatency_Pipeline(Constants.LL_INTAKE);
    inputs.aCAPTURELATENCY = LimelightHelpers.getLatency_Capture(Constants.LL_INTAKE);
    inputs.aTHOR =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "thor").getDouble(0.0);
    inputs.aTVERT =
        LimelightHelpers.getLimelightNTTableEntry(Constants.LL_INTAKE, "tvert").getDouble(0.0);
  }
}
