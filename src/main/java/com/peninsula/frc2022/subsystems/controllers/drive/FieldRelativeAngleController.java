package com.peninsula.frc2022.subsystems.controllers.drive;

import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldRelativeAngleController extends Swerve.SwerveController {

  PIDController angleSetPIDController;

  public FieldRelativeAngleController() {
    angleSetPIDController =
        new PIDController(
            SwerveConstants.kSetAngleGains.p,
            SwerveConstants.kSetAngleGains.i,
            SwerveConstants.kSetAngleGains.d);
    angleSetPIDController.enableContinuousInput(0, 360);
  }

  // Override this for shoot on move controller
  public double getAngle(Commands commands, RobotState state) {
    return commands.angleWanted;
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
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
        angleSetPIDController.calculate(state.gyroHeading.getDegrees(), getAngle(commands, state));

    SmartDashboard.putNumber("angleW", getAngle(commands, state));

    SwerveModuleState[] moduleStates =
        SwerveConstants.kKinematics.toSwerveModuleStates(
            commands.robotCentricWanted
                ? new ChassisSpeeds(x, y, z)
                : ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, state.gyroHeading));

    normalize(moduleStates);
    mOutputs.setOutputs(moduleStates);
  }

  public void normalize(SwerveModuleState[] swerveModuleStates) {
    boolean shouldNormalize = false;
    double maxModuleVelocity = 0;
    for (SwerveModuleState state : swerveModuleStates) {
      if (state.speedMetersPerSecond > SwerveConstants.kMaxVelocityMetersPerSecond) {
        shouldNormalize = true;
        maxModuleVelocity = Math.max(maxModuleVelocity, state.speedMetersPerSecond);
      }
    }

    if (shouldNormalize) {
      for (SwerveModuleState state : swerveModuleStates) {
        state.speedMetersPerSecond *=
            SwerveConstants.kMaxVelocityMetersPerSecond / maxModuleVelocity;
      }
    }
  }
}
