package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.LimelightHelpers;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public LimelightHelpers.PoseEstimate visionPose = new LimelightHelpers.PoseEstimate(null, -1, 0, 0, 0, 0, 0, null);
        public TimestampedT2d robotRelNotePoseCorrected = new TimestampedT2d(new Translation2d(), -1);
        public Translation2d robotRelNotePoseRaw = new Translation2d();

        public boolean aprilTagLimelightConnected = false;
        public boolean intakeLimelightConnected = false;
    }

    public class TimestampedT2d {
        Translation2d translation;
        double time;

    public TimestampedT2d(Translation2d translation, double time) {
      this.translation = translation;
      this.time = time;
    }
  }

    public default void updateInputs(VisionIOInputs inputs) {}
}
