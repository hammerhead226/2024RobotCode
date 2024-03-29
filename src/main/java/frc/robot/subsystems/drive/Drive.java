// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Constants.SwerveConstants.MAX_LINEAR_SPEED * 0.8;
  private static final double TRACK_WIDTH_X = Constants.SwerveConstants.TRACK_WIDTH_X;
  private static final double TRACK_WIDTH_Y = Constants.SwerveConstants.TRACK_WIDTH_Y;
  private static final double DRIVE_BASE_RADIUS = Constants.SwerveConstants.DRIVE_BASE_RADIUS;
  private static final double MAX_ANGULAR_SPEED = Constants.SwerveConstants.MAX_ANGULAR_SPEED;
  private static double multiplier = 1.0;
  private static boolean toggle = false;

  private NetworkTable limelightintake =
      NetworkTableInstance.getDefault().getTable(Constants.LL_INTAKE);

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private final PIDController rotationController;

  private double timeSinceLastCorrection = 0;
  private double lasttime = 0;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  TimestampedT2d lastNoteLocT2d = new TimestampedT2d(new Translation2d(0, 0), -1.);

  public class TimestampedPose2d {
    Pose2d pose;
    double time;
  }

  public class TimestampedT2d {
    Translation2d translation;
    double time;

    public TimestampedT2d(Translation2d translation, double time) {
      this.translation = translation;
      this.time = time;
    }
  }

  CircularBuffer<TimestampedPose2d> robotPoseBuffer;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5),
            new PIDConstants(1.5),
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    lasttime = Timer.getFPGATimestamp();

    rotationController = new PIDController(0.1, 0, 0);

    rotationController.setTolerance(5);
    rotationController.enableContinuousInput(-180, 180);
    robotPoseBuffer = new CircularBuffer<>(11);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);

    if (DriverStation.getAlliance().isPresent() && LimelightHelpers.getTV(Constants.LL_ALIGN)) {
      visionLogic();
    }

    Logger.recordOutput(
        "limelilght alig latency", LimelightHelpers.getLatency_Pipeline(Constants.LL_ALIGN));
    Logger.recordOutput("limelight intake hb", limelightintake.getEntry("hb").getDouble(0.0));

    // Logger.recordOutput("note time", getCachedNoteTime());
    // Note Pose estimating
    updatePoseBuffer();
    if (LimelightHelpers.getTX(Constants.LL_INTAKE) != 0.0) {
      double taThreshold = 0;
      if (LimelightHelpers.getTA(Constants.LL_INTAKE) >= taThreshold) {
        lastNoteLocT2d.translation = calculateNotePositionFieldRelative();
        lastNoteLocT2d.time = Timer.getFPGATimestamp();
      }
    }

    Logger.recordOutput(
        "rotation for note",
        new Rotation2d(
                getCachedNoteLocation().getX() - getPose().getX(),
                getCachedNoteLocation().getY() - getPose().getY())
            .getDegrees());
  }

  private void updatePoseBuffer() {
    TimestampedPose2d now = new TimestampedPose2d();
    now.pose = getPose();
    now.time = Timer.getFPGATimestamp();
    robotPoseBuffer.addFirst(now);
  }

  private Pose2d posePicker(double time) {
    TimestampedPose2d prev = robotPoseBuffer.getFirst();
    for (int i = 0; i < robotPoseBuffer.size(); i++) {
      TimestampedPose2d next = robotPoseBuffer.get(i);
      double delta = next.time - time;
      if (delta < 0) {
        double t = ((time - next.time) / (prev.time - next.time));
        return next.pose.interpolate(prev.pose, t);
      }
    }
    // if the time is before everything in the buffer return the oldest thing
    return robotPoseBuffer.getLast().pose;
  }

  public void visionLogic() {
    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LL_ALIGN);

    double xMeterStds;
    double yMeterStds;
    double headingDegStds;

    double poseDifference = getVisionPoseDifference(limelightMeasurement.pose);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Logger.recordOutput("avg area", limelightMeasurement.avgTagArea);

    if (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagArea > 0.04) {
      xMeterStds = 0.7;
      yMeterStds = 0.7;
      headingDegStds = 8;
    } else if (limelightMeasurement.tagCount == 1 && poseDifference < 0.5) {
      xMeterStds = 5;
      yMeterStds = 5;
      headingDegStds = 30;
    } else if (limelightMeasurement.tagCount == 1 && poseDifference < 3) {
      xMeterStds = 11.43;
      yMeterStds = 11.43;
      headingDegStds = 9999;
    } else return;

    Logger.recordOutput("number of tags", limelightMeasurement.tagCount);

    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(xMeterStds, yMeterStds, Units.degreesToRadians(headingDegStds)));

    Pose2d pose = limelightMeasurement.pose;

    if (isFlipped) {
      pose.getRotation().plus(new Rotation2d(Math.PI));
    }

    Logger.recordOutput("Vision Measurement", limelightMeasurement.pose);

    addVisionMeasurement(
        pose, limelightMeasurement.timestampSeconds - (limelightMeasurement.latency / 1000.));
  }

  public double getVisionPoseDifference(Pose2d visionPose) {
    return getPose().getTranslation().getDistance(visionPose.getTranslation());
  }

  public boolean acceptableMeasurements(Pose2d visionMeasurement) {
    return Math.abs(visionMeasurement.getX() - getPose().getX()) < 1
        && Math.abs(visionMeasurement.getY() - getPose().getY()) < 1;
  }

  public boolean canCorrect(Pose2d visionMeasurement, double timeSinceLastCorrection) {
    if (timeSinceLastCorrection < 5) {
      if (acceptableMeasurements(visionMeasurement)) return true;
    } else {
      return true;
    }
    return false;
  }

  public void toggleLowSpeed() {
    if (!toggle) {
      multiplier = 0.4;
    } else {
      multiplier = 1;
    }
    toggle = !toggle;
  }

  public void driveTo(Translation2d coord, Rotation2d angle) {
    Translation2d translationErr = coord.minus(getPose().getTranslation());

    Rotation2d rotationErr = angle.minus(getPose().getRotation());

    rotationController.setSetpoint(angle.getDegrees());

    Logger.recordOutput("note location err", translationErr);

    Translation2d power = new Translation2d(translationErr.getNorm(), translationErr.getAngle());

    Rotation2d rotPower = new Rotation2d(rotationErr.getRadians());

    double angularSpeed = rotationController.calculate(getPose().getRotation().getDegrees());

    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(-power.getX(), -power.getY(), rotPower.getDegrees()),
            rawGyroRotation));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED * multiplier);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Resets the current odometry pose. */
  // public void setGyroPose(Pose2d pose) {
  //   if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
  //     // pose.getRotation().plus(new Rotation2d(Math.PI));
  //     this.rawGyroRotation = rawGyroRotation.plus(new Rotation2d(Math.PI));
  //   }
  //   Logger.recordOutput("reset pose", pose);
  //   poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  // }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public double getNoteError() {
    return getIntakeLLTx();
  }

  public double getIntakeLLTx() {
    return LimelightHelpers.getTX("limelight-intake");
  }

  public double getIntakeLLTy() {
    return LimelightHelpers.getTY("limelight-intake");
  }

  public Translation2d calculateNotePositionFieldRelative() {

    double distInch = (1 / (40 - ((30) * getIntakeLLTy() / 23)) * 1000); // Convert degrees to inch
    double noteYawAngleDeg = -getIntakeLLTx() - 3; // account for static offset, reverse to be CCW+
    double radiusInch = distInch / Math.cos(Units.degreesToRadians(noteYawAngleDeg));
    Logger.recordOutput("NoteTracking/distInch", distInch);
    Logger.recordOutput("NoteTracking/noteYawAngleDeg", noteYawAngleDeg);
    Logger.recordOutput("NoteTracking/radius", radiusInch);

    // camera relative -> bot relative -> field relative
    Translation2d camRelNoteLocT2d =
        new Translation2d(
            Units.inchesToMeters(radiusInch), Rotation2d.fromDegrees(noteYawAngleDeg));
    Logger.recordOutput("NoteTracking/camRelNoteLocT2d", camRelNoteLocT2d);

    Translation2d roboRelNoteLocT2d =
        camRelNoteLocT2d
            .rotateBy(Rotation2d.fromDegrees(0))
            .plus(new Translation2d(Units.inchesToMeters(12), 0));
    Logger.recordOutput("NoteTracking/roboRelNoteLocT2d", roboRelNoteLocT2d);
    Pose2d pickedRobotPose =
        posePicker(
            Timer.getFPGATimestamp()
                - LimelightHelpers.getLatency_Pipeline(Constants.LL_INTAKE)
                - LimelightHelpers.getLatency_Capture(Constants.LL_INTAKE));
    Translation2d fieldRelNoteLocT2d =
        roboRelNoteLocT2d
            .rotateBy(pickedRobotPose.getRotation())
            .plus(pickedRobotPose.getTranslation());
    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2d", fieldRelNoteLocT2d);
    Logger.recordOutput(
        "distance from center of robot",
        Units.metersToInches(fieldRelNoteLocT2d.getDistance(getPose().getTranslation())));
    return fieldRelNoteLocT2d;
  }

  public Translation2d getCachedNoteLocation() {
    // if (lastNoteLocT2d.translation.equals(null)) {
    // return new Translation2d(0, 0);
    // } else {
    return lastNoteLocT2d.translation;
    // }
  }

  public boolean noteImageIsNew() {
    double maxNoteAge = 1.0; // seconds
    return Timer.getFPGATimestamp() - lastNoteLocT2d.time < maxNoteAge;
  }

  public double getCachedNoteTime() {
    return lastNoteLocT2d.time;
  }

  public PathPlannerPath generateTrajectory(
      Pose2d target,
      double maxVelMetersPerSec,
      double maxAccelMetersPerSecSquared,
      double maxAngVelDegPerSec,
      double maxAngAccelDegPerSecSquared,
      double endVelMetersPerSec) {
    List<Translation2d> points =
        PathPlannerPath.bezierFromPoses(
            new Pose2d(getPose().getY(), getPose().getX(), getPose().getRotation()),
            new Pose2d(target.getY(), target.getX(), target.getRotation()));
    PathPlannerPath path =
        new PathPlannerPath(
            points,
            new PathConstraints(
                maxAngVelDegPerSec,
                maxAccelMetersPerSecSquared,
                Units.degreesToRadians(maxAngVelDegPerSec),
                Units.degreesToRadians(maxAngAccelDegPerSecSquared)),
            new GoalEndState(endVelMetersPerSec, target.getRotation(), true));

    return path;
  }

  public void runPath(PathPlannerPath path) {
    AutoBuilder.followPath(path).schedule();
  }

  public void AlignToNote(Intake intake, Pivot pivot, Shooter shooter, LED led) {
    intake.runRollers(12);
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
    shooter.setFeedersRPM(500);

    Rotation2d targetRotation;
    Logger.recordOutput("note timeess", getCachedNoteTime());
    if (getCachedNoteTime() != -1) {
      Translation2d cachedNoteT2d = getCachedNoteLocation();
      Logger.recordOutput("better translate", cachedNoteT2d);
      if (noteImageIsNew()) {

        targetRotation =
            new Rotation2d(
                cachedNoteT2d.getX() - getPose().getX(), cachedNoteT2d.getY() - getPose().getY());
        List<Translation2d> pointsToNote;
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
          Logger.recordOutput(
              "goal point blue",
              new Pose2d(cachedNoteT2d.getX(), cachedNoteT2d.getY(), targetRotation));
          pointsToNote =
              PathPlannerPath.bezierFromPoses(
                  new Pose2d(getPose().getX(), getPose().getY(), getPose().getRotation()),
                  new Pose2d(cachedNoteT2d.getX(), cachedNoteT2d.getY(), targetRotation));
        } else {
          Logger.recordOutput(
              "goal point red",
              new Pose2d(cachedNoteT2d.getX(), cachedNoteT2d.getY(), targetRotation));
          pointsToNote =
              PathPlannerPath.bezierFromPoses(
                  new Pose2d(
                      FieldConstants.fieldLength - getPose().getX(),
                      getPose().getY(),
                      getPose().getRotation()),
                  new Pose2d(cachedNoteT2d.getX(), cachedNoteT2d.getY(), targetRotation));
        }
        PathPlannerPath path =
            new PathPlannerPath(
                pointsToNote,
                new PathConstraints(
                    2, 1.5, Units.degreesToRadians(100), Units.degreesToRadians(180)),
                new GoalEndState(0.5, targetRotation, true));
        // PathPlannerLogging.logActivePath(path);
        // return AutoBuilder.followPath(path);
        // Logger.recordOutput("epic path", path);
        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();
        // return new InstantCommand(() -> led.setState(LED_STATE.RED));

      } else {
        // return new InstantCommand(() -> led.setState(LED_STATE.FLASHING_RED));
      }
    } else {
      // return new InstantCommand(() -> led.setState(LED_STATE.FLASHING_GREEN));
    }
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}
