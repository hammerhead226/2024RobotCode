package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.LimelightHelpers;

public interface VisionIO {

    static class RawLimelightData {
      public double tx;
      public double ty;
      public double ta;
      public double hb;
      public boolean tv;

      public RawLimelightData() {
        
      }
    }

    static class NoteData {
      public double timestamp;
      public double distanceInches;
      public double yawDegs;

      public NoteData(double timestamp, double distanceInches, double yawDegs) {
        this.timestamp = timestamp;
        this.distanceInches = distanceInches;
        this.yawDegs = yawDegs;
      }

      public NoteData() {}
      
    }
    @AutoLog
    public static class VisionIOInputs {
        public LimelightHelpers.PoseEstimate visionPose = new LimelightHelpers.PoseEstimate(null, -1, 0, 0, 0, 0, 0, null);
        public NoteData noteData = new NoteData();

        public double currentTime;

        public boolean aprilTagLimelightConnected = false;
        public boolean intakeLimelightConnected = false;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
