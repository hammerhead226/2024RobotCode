package frc.robot.subsystems.vision;

import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {


  @AutoLog
  public static class VisionIOInputs {
    public LimelightHelpers.PoseEstimate visionPose =
        new LimelightHelpers.PoseEstimate(null, -1, 0, 0, 0, 0, 0, null);

    public double iTX;
    public double iTY;
    public double iTA;
    public double iHB;
    public boolean iTV;

    public double iTHOR;
    public double iTVERT;

    public double iPIPELINELATENCY;
    public double iCAPTURELATENCY;

    public double aTX;
    public double aTY;
    public double aTA;
    public double aHB;
    public boolean aTV;

    public double aTHOR;
    public double aTVERT;

    public double aPIPELINELATENCY;
    public double aCAPTURELATENCY;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
