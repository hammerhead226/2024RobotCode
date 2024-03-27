package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIONote implements VisionIO {
  private double angleDegs;
  private double distanceMeters;

  public VisionIONote() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.angleDegs = LimelightHelpers.getTX(Constants.LL_INTAKE);
  }
}
