package com.peninsula.frc2022.subsystems.controllers.drive;

import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Swerve;
import com.peninsula.frc2022.util.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class AlignController extends Swerve.SwerveController {

  PIDController farAlignPidController;
  PIDController closeAlignPidController;
  TimeInterpolatableBuffer visionTemporalPoses;

  public AlignController() {
    farAlignPidController =
        new PIDController(
            SwerveConstants.kCloseAlignGains.p,
            SwerveConstants.kCloseAlignGains.i,
            SwerveConstants.kCloseAlignGains.d);
    closeAlignPidController =
        new PIDController(
            SwerveConstants.kCloseAlignGains.p,
            SwerveConstants.kCloseAlignGains.i,
            SwerveConstants.kCloseAlignGains.d);
    visionTemporalPoses = TimeInterpolatableBuffer.createDoubleBuffer(2);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    visionTemporalPoses.addSample(
        Timer.getFPGATimestamp(), state.targetYawRad); // robot to goal centric pose
    double latencyCompGoalPose =
        (double)
            visionTemporalPoses.getSample(
                Timer.getFPGATimestamp() - state.visionLatencyS); // robot centric to goal;
    double x =
        state.driverLeftY
            * (commands.boostWanted
                ? SwerveConstants.kTeleopBoostMaxTransVel
                : SwerveConstants.kTeleopMaxTransVel);
    double y =
        state.driverLeftX
            * (commands.boostWanted
                ? SwerveConstants.kTeleopBoostMaxTransVel
                : SwerveConstants.kTeleopMaxTransVel);
    double z =
        -(closeAlignPidController.calculate(latencyCompGoalPose)
            + Math.copySign(0.2, closeAlignPidController.getPositionError()));
    SwerveModuleState[] moduleStates =
        SwerveConstants.kKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, state.gyroHeading));
    mOutputs.setOutputs(moduleStates);
    mRumbleOutput =
        Math.abs(state.targetYawRad) < Math.PI;
  }
}
