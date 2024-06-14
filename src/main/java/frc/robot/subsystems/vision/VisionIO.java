package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public Pose2d visionPose = new Pose2d();
    public double timestampSeconds;
    public int tagCount;
    public double latency;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;

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
