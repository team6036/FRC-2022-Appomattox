package com.peninsula.frc2022.robot;

import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.config.VisionConstants;
import com.peninsula.frc2022.robot.HardwareAdapter.JoystickHardware;
import com.peninsula.frc2022.subsystems.*;
import com.peninsula.frc2022.util.ShootingOnMoveSolver;
import com.peninsula.frc2022.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;

public class HardwareReader {

  public HardwareReader() {}

  /**
   * Takes all of the sensor data from the hardware, and unwraps it into the current {@link
   * RobotState}.
   */
  void readState(Set<SubsystemBase> enabledSubsystems, RobotState state) {
    readGameAndFieldState(state);
    Robot.sLoopDebugger.addPoint("readField");
    if (state.gamePeriod == RobotState.GamePeriod.TELEOP) readJoystickState(state);
    Robot.sLoopDebugger.addPoint("readJoy");
    if (enabledSubsystems.contains(Swerve.getInstance())) readSwerveState(state);
    Robot.sLoopDebugger.addPoint("readSwerve");
    if (enabledSubsystems.contains(Indexer.getInstance())) readIndexerState(state);
    Robot.sLoopDebugger.addPoint("readIndexer");
    if (enabledSubsystems.contains(Shooter.getInstance())) readShooterState(state);
    Robot.sLoopDebugger.addPoint("readShooter");
    if (enabledSubsystems.contains(Climber.getInstance())) readClimberState(state);
    Robot.sLoopDebugger.addPoint("readClimb");
    if (enabledSubsystems.contains(Intake.getInstance())) readIntakeState(state);
    Robot.sLoopDebugger.addPoint("readIntake");

    if (enabledSubsystems.contains(Vision.getInstance())) {
      readVisionState(state);
      Robot.sLoopDebugger.addPoint("readVision");
    }
  }

  private void readGameAndFieldState(RobotState state) {
    state.cycles += 1;
    state.gameTimeS = Timer.getFPGATimestamp();
  }

  public void readColorAndGameData(RobotState state) {
    state.gameData = DriverStation.getGameSpecificMessage();
    state.allianceColor = DriverStation.getAlliance();
  }

  /** Reads the arm position of intake */
  private void readIntakeState(RobotState state) {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();
    state.intakeArmPosition = hardware.armMotorRight.getSelectedSensorPosition();
    state.intakeArmRadians = state.intakeArmPosition / (30 * 2048) * 6.2831853072;
    SmartDashboard.putNumber("Intake Arm", state.intakeArmPosition);
  }

  /** Reads swerve module states (8 motors) Updates odometry + solves virtual goal for SoM */
  private void readSwerveState(RobotState state) {
    var hardware = HardwareAdapter.SwerveHardware.getInstance();
    var miscHardware = HardwareAdapter.MiscHardware.getInstance();
    state.gyroHeading = Rotation2d.fromDegrees(360.0 - miscHardware.gyro.getYaw());
    for (int i = 0; i < state.moduleEncoderPos.length; i++) {
      state.moduleEncoderPos[i] = hardware.modules[i].getSteerAngle();
    }

    var outputs = Swerve.getInstance().getOutputs().getStates();

    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) moduleStates[i] = new SwerveModuleState();

    moduleStates[0].angle = outputs[0].angle;
    moduleStates[0].speedMetersPerSecond = Math.abs(hardware.modules[0].getDriveVelocity());

    moduleStates[1].angle = outputs[1].angle;
    moduleStates[1].speedMetersPerSecond = Math.abs(hardware.modules[1].getDriveVelocity());

    moduleStates[2].angle = outputs[2].angle;
    moduleStates[2].speedMetersPerSecond = Math.abs(hardware.modules[2].getDriveVelocity());

    moduleStates[3].angle = outputs[3].angle;
    moduleStates[3].speedMetersPerSecond = Math.abs(hardware.modules[3].getDriveVelocity());

    state.gyroHeading = state.gyroHeading.plus(state.initPose.getRotation());
    state.gyroHeading = Rotation2d.fromDegrees((state.gyroHeading.getDegrees() + 720) % 360);
    SmartDashboard.putNumber("angleC", state.gyroHeading.getDegrees());

    // +x is shooter side, +y is side with PDP
    state.chassisRelativeSpeeds = SwerveConstants.kKinematics.toChassisSpeeds(moduleStates);

    // Velocity of robot on the field coordinate system
    state.fieldRelativeSpeeds =
        new ChassisSpeeds(
            state.gyroHeading.getCos() * state.chassisRelativeSpeeds.vxMetersPerSecond
                - state.gyroHeading.getSin() * state.chassisRelativeSpeeds.vyMetersPerSecond,
            state.gyroHeading.getSin() * state.chassisRelativeSpeeds.vxMetersPerSecond
                + state.gyroHeading.getCos() * state.chassisRelativeSpeeds.vyMetersPerSecond,
            state.chassisRelativeSpeeds.omegaRadiansPerSecond);

    state.driveOdometry.update(state.gyroHeading, moduleStates);
    state.odometryDeadReckonUpdateTime = state.gameTimeS;
    state.pastPoses.addSample(
        state.odometryDeadReckonUpdateTime, state.driveOdometry.getPoseMeters());

    virtualGoalUpdater(state);
  }

  public void virtualGoalUpdater(RobotState state) {
    // virtual goal translation from robot
    Translation2d targetToRobot = state.realPose.minus(VisionConstants.target).getTranslation();

    state.shotDistanceCurrent = targetToRobot.getNorm();

    // Moves goal by a solved shot time in the negative velocity direction
    double shotTime =
        ShootingOnMoveSolver.ToF(
            state.shotDistanceCurrent,
            state.fieldRelativeSpeeds.vxMetersPerSecond,
            state.fieldRelativeSpeeds.vyMetersPerSecond,
            state.realPose.getX(),
            state.realPose.getY());

    double xVirtualMove = -state.fieldRelativeSpeeds.vxMetersPerSecond * shotTime;
    double yVirtualMove = -state.fieldRelativeSpeeds.vyMetersPerSecond * shotTime;

    state.virtualGoal =
        VisionConstants.target.transformBy(
            new Transform2d(new Translation2d(xVirtualMove, yVirtualMove), new Rotation2d(0)));
  }

  /** Reads 2 xbox controllers */
  private void readJoystickState(RobotState state) {
    var hardware = JoystickHardware.getInstance();

    state.driverLeftX = Util.handleDeadBand(hardware.driverXboxController.getLeftX(), 0.00);
    state.driverLeftY = Util.handleDeadBand(hardware.driverXboxController.getLeftY(), 0.00);
    state.driverRightX = Util.handleDeadBand(hardware.driverXboxController.getRightX(), 0.00);
    state.driverRt = hardware.driverXboxController.getRightTriggerAxis();

    state.operatorAPressed = hardware.operatorXboxController.getAButton();

    state.operatorRtPressed = hardware.operatorXboxController.getRightTriggerAxis() > 0.5;
    state.operatorLtPressed = hardware.operatorXboxController.getLeftTriggerAxis() > 0.5;
    state.operatorRbPressed = hardware.operatorXboxController.getRightBumper();
    state.operatorLbPressed = hardware.operatorXboxController.getLeftBumper();
    state.operatorDPadLeftPressed = hardware.operatorXboxController.getLeftStickButton();
    state.operatorDPadRightPressed = hardware.operatorXboxController.getRightStickButton();
    state.driverAPressed = hardware.driverXboxController.getAButton();
    state.driverRbPressed = hardware.driverXboxController.getRightBumper();
    state.operatorLeftX = Util.handleDeadBand(hardware.operatorXboxController.getLeftX(), 0.09);
    state.operatorLeftY = Util.handleDeadBand(hardware.operatorXboxController.getLeftY(), 0.09);

    state.operatorRightX = Util.handleDeadBand(hardware.operatorXboxController.getRightX(), 0.03);
    state.operatorRightY = Util.handleDeadBand(hardware.operatorXboxController.getRightY(), 0.03);

    state.operatorAPressed = hardware.operatorXboxController.getAButton();
    state.operatorBPressed = hardware.operatorXboxController.getBButton();
    state.operatorXPressed = hardware.operatorXboxController.getXButton();
    state.operatorYPressed = hardware.operatorXboxController.getYButton();
    state.driverLtPressed = hardware.driverXboxController.getLeftTriggerAxis() > 0.5;

    state.driverLBPressed = hardware.driverXboxController.getLeftBumper();
    state.driverXPressed = hardware.driverXboxController.getXButton();
    state.driverYPressed = hardware.driverXboxController.getYButton();
    state.driverBPressed = hardware.driverXboxController.getBButton();
  }

  /** Reads position of indexer wheel Reads 3 IR sensors for ball state estimation */
  private void readIndexerState(RobotState state) {
    var hardware = HardwareAdapter.IndexerHardware.getInstance();
    state.kickerPosition = hardware.kickerMotor.getSelectedSensorPosition();
    state.firstIRBlocked = !hardware.intakeIndexerSensor.get();
    state.lastIRBlocked = !hardware.stopperSensor.get();
    state.secondIRBlocked = !hardware.firstPosSensor.get();
  }

  /** Reads data from photonvision in network tables and performs pose estimation calculations */
  private void readVisionState(RobotState state) {
    var hardware = HardwareAdapter.VisionHardware.getInstance();
    try {
      state.targetYawRad =
          hardware.camera.getLatestResult().getTargets().get(0).getYaw() * Math.PI / 180.0;
      state.targetPitchRad =
          hardware.camera.getLatestResult().getTargets().get(0).getPitch() * Math.PI / 180.0;

      double x, y, z = 1;
      y = Math.tan(state.targetPitchRad) * z;
      x = Math.tan(state.targetYawRad) * z;

      double norm = Math.sqrt(x * x + y * y + z * z);
      x /= norm;
      y /= norm;
      z /= norm;

      // Rotate the vector by the camera pitch
      Translation2d yzPrime =
          new Translation2d(y, z).rotateBy(new Rotation2d(-VisionConstants.cameraPitchRad));
      double yPrime = yzPrime.getX();

      // Solve for the intersection
      double angleToGoalRadians = Math.asin(yPrime / 1); // redid 5940 math
      double diffHeight = VisionConstants.targetheightM - VisionConstants.camHeightM;
      state.targetDistM = diffHeight / Math.tan(angleToGoalRadians) + VisionConstants.targetwidthM;

      SmartDashboard.putNumber("distV", state.targetDistM);

      state.visionLatencyS =
          (hardware.camera.getLatestResult().getLatencyMillis() + 11)
              / 1000.0; // 11ms for snapshot time

      if (state.useForPoseEstimation) {
        state.lastVisionEstimatedPose =
            new Pose2d(
                new Translation2d(state.targetDistM, 0)
                    .rotateBy(
                        state
                            .gyroHeading
                            .plus(Rotation2d.fromDegrees(180))
                            .minus(new Rotation2d(state.targetYawRad))),
                state.gyroHeading);

        state.lastVisionEstimatedPose =
            new Pose2d(
                state.lastVisionEstimatedPose.getX() + VisionConstants.target.getX(),
                state.lastVisionEstimatedPose.getY() + VisionConstants.target.getY(),
                state.lastVisionEstimatedPose.getRotation());

        state.odometryVisionUpdateTime = state.gameTimeS - state.visionLatencyS;
      }

    } catch (Exception e) {
    }

    state.targetSeen = hardware.camera.getLatestResult().hasTargets();

    Pose2d odomPoseAtSnapshot = state.pastPoses.getSample(state.odometryVisionUpdateTime);

    // Adds delta pose from odometry back to vision truth pose
    state.realPose =
        new Pose2d(
            state.lastVisionEstimatedPose.getX()
                + state.pastPoses.getSample(state.gameTimeS).getX()
                - odomPoseAtSnapshot.getX(),
            state.lastVisionEstimatedPose.getY()
                + state.pastPoses.getSample(state.gameTimeS).getY()
                - odomPoseAtSnapshot.getY(),
            state.gyroHeading);
  }

  /** Reads velocity of shooter wheel */
  private void readShooterState(RobotState state) {
    var hardware = HardwareAdapter.ShooterHardware.getInstance();
    state.mainvel = hardware.shooterMasterMotor.getSelectedSensorVelocity(0) * 600.0 / 2048.0;
  }

  /**
   * Reads velocities, currents and positions of 2 climber arms Read gyro pitch for the tilt of the
   * robot
   */
  private void readClimberState(RobotState state) {
    var hardware = HardwareAdapter.ClimberHardware.getInstance();
    state.rightTick = hardware.rightPuller.getSelectedSensorPosition();
    state.leftTick = hardware.leftPuller.getSelectedSensorPosition();

    state.rightTickVelo = hardware.rightPuller.getSelectedSensorVelocity();
    state.leftTickVelo = hardware.leftPuller.getSelectedSensorVelocity();

    state.rightStatorCurrent = hardware.rightPuller.getStatorCurrent();
    state.leftStatorCurrent = hardware.leftPuller.getStatorCurrent();

    SmartDashboard.putNumber("rTicksC", state.rightTick);
    SmartDashboard.putNumber("lTicksC", state.leftTick);

    state.gyroPitch =
        Rotation2d.fromDegrees(HardwareAdapter.MiscHardware.getInstance().gyro.getPitch());
  }
}
