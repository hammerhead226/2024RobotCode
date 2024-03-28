package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionIONote implements VisionIO {
  public VisionIONote() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.angleDegs = LimelightHelpers.getTX(Constants.LL_INTAKE);

    inputs.distanceMeters = getY() / Math.cos(LimelightHelpers.getTX(Constants.LL_INTAKE));
  }

  private double getY() {
    // might need to mess around with intake ll angle if ty is -
    double theta =
        Math.abs(Constants.INTAKE_LL_ANGLE + LimelightHelpers.getTY(Constants.LL_INTAKE));
    return Constants.INTAKE_LL_HEIGHT_METERS / Math.atan(theta);
  }
}
