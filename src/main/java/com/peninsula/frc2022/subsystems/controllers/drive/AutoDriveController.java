package com.peninsula.frc2022.subsystems.controllers.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Swerve;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveController extends Swerve.SwerveController {

  private final Timer mTimer = new Timer();
  private HolonomicDriveController mTrajectoryController =
      new HolonomicDriveController(
          new PIDController(1.04, 0.0, 0.0),
          new PIDController(1.04, 0.0, 0.0),
          new ProfiledPIDController(
              2.0, 0.00, 0.00, new TrapezoidProfile.Constraints(5 * Math.PI, Math.PI * 10)));

  PathPlannerTrajectory mPathPlannerTrajectory;

  public AutoDriveController() {
    mTimer.start();
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {

    PathPlannerTrajectory mPathPlannerTrajectoryWanted =
        commands.getDriveWantedPathPlannerTrajectory();

    if (mPathPlannerTrajectory != mPathPlannerTrajectoryWanted) {
      mPathPlannerTrajectory = mPathPlannerTrajectoryWanted;

      ProfiledPIDController thetaController =
          new ProfiledPIDController(
              5.0, 0.00, 0.00, new TrapezoidProfile.Constraints(3 * Math.PI, 9 * Math.PI));

      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      mTrajectoryController =
          new HolonomicDriveController(
              new PIDController(2.14, 0.0, 0.0),
              new PIDController(1.94, 0.0, 0.0),
              thetaController);
      mTimer.reset();
    }

    PathPlannerTrajectory.PathPlannerState targetPose =
        (PathPlannerTrajectory.PathPlannerState) mPathPlannerTrajectory.sample(mTimer.get());

    ChassisSpeeds targetSpeeds =
        mTrajectoryController.calculate(
            state.driveOdometry.getPoseMeters(), targetPose, targetPose.holonomicRotation);

    targetSpeeds.omegaRadiansPerSecond = -targetSpeeds.omegaRadiansPerSecond;

    SwerveModuleState[] moduleStates =
        SwerveConstants.kKinematics.toSwerveModuleStates(targetSpeeds);

    mOutputs.setOutputs(moduleStates, targetPose.holonomicRotation);
  }
}
