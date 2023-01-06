package com.peninsula.frc2022.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.config.VisionConstants;
import com.peninsula.frc2022.util.Util;
import com.peninsula.frc2022.util.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;

/** Holds the current physical state of the robot from our sensors. */
@SuppressWarnings("java:S1104")
public class RobotState {

  public enum GamePeriod {
    AUTO,
    TELEOP,
    TESTING,
    DISABLED
  }

  public static final String kLoggerTag = Util.classToJsonName(RobotState.class);

  /* Swerve */
  public Rotation2d gyroHeading = new Rotation2d();
  public Rotation2d gyroPitch = new Rotation2d();

  public double[] moduleEncoderPos = new double[4];
  public double avgModuleVel;

  public SwerveDriveOdometry driveOdometry =
      new SwerveDriveOdometry(SwerveConstants.kKinematics, new Rotation2d(0));

  public ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
  public ChassisSpeeds chassisRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

  public TimeInterpolatableBuffer<Pose2d> pastPoses = TimeInterpolatableBuffer.createBuffer(1.0);

  /* Climb */
  public double rightTick, leftTick;
  public double rightTickVelo, leftTickVelo;
  public double rightStatorCurrent, leftStatorCurrent;

  /* Positioning Vision */
  public Pose2d virtualGoal =
      new Pose2d(VisionConstants.target.getX(), VisionConstants.target.getY(), new Rotation2d(0));

  public Pose2d realPose = new Pose2d(0, 0, Rotation2d.fromDegrees(-88 + 360));

  public Pose2d lastVisionEstimatedPose = new Pose2d();
  public double odometryDeadReckonUpdateTime = 0;
  public double odometryVisionUpdateTime = 0;

  /* Indexer */
  public boolean firstIRBlocked, lastIRBlocked, secondIRBlocked;
  public double kickerPosition;

  /* Intake */
  public double intakeArmPosition;
  public double intakeArmRadians;

  /* Shooter */
  public double mainvel = 0;
  public double shotDistanceCurrent;

  /* Climber */

  /* Joystick */
  public double driverLeftX, driverRightX, driverLeftY, driverRt = 0;
  public boolean operatorAPressed,
      operatorBPressed,
      operatorXPressed,
      operatorYPressed,
      operatorRtPressed,
      operatorLtPressed,
      operatorRbPressed,
      operatorLbPressed,
      operatorDPadLeftPressed,
      operatorDPadRightPressed;
  public double operatorLeftX, operatorLeftY, operatorRightY, operatorRightX;
  public boolean driverAPressed, driverRbPressed;
  public boolean driverLtPressed;

  public boolean driverBPressed;
  public boolean driverLBPressed;
  public boolean driverXPressed;
  public boolean driverYPressed;

  /* Miscellaneous */
  public GamePeriod gamePeriod = GamePeriod.DISABLED;
  public String gameData;
  public int cycles = 0;
  public double gameTimeS = 0;
  public double pitchSet = 0;

  /* Vision */
  public double targetYawRad, targetPitchRad, targetDistM, visionLatencyS;
  public boolean targetSeen = false;
  public boolean useForPoseEstimation = true;

  /* Auto */
  public PathPlannerTrajectory currentTrajectory;
  public Pose2d initPose = new Pose2d(0, 0, Rotation2d.fromDegrees(-88 + 360));

  public DriverStation.Alliance allianceColor;
}
