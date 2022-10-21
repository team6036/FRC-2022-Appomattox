package com.peninsula.frc2022.subsystems.controllers.drive;

import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootOnMoveController extends FieldRelativeAngleController {

  public ShootOnMoveController() {
    slewStart = false;
  }

  boolean slewStart;
  SlewRateLimiter slewX = new SlewRateLimiter(0, 0), slewY = new SlewRateLimiter(0, 0);

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

    if (!slewStart) {
      slewStart = true;
      slewX = new SlewRateLimiter(0.2, x);
      slewY = new SlewRateLimiter(0.2, y);
    }

    SmartDashboard.putNumber("angleW", getAngle(commands, state));

    SwerveModuleState[] moduleStates =
        SwerveConstants.kKinematics.toSwerveModuleStates(
            commands.robotCentricWanted
                ? new ChassisSpeeds(slewX.calculate(x), slewY.calculate(y), z)
                : ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, state.gyroHeading));

    normalize(moduleStates);
    mOutputs.setOutputs(moduleStates);
  }

  @Override
  public double getAngle(Commands commands, RobotState state) {

    return (new Rotation2d(
                    Math.atan2(
                        -state.realPose.getY() + state.virtualGoal.getY(),
                        -state.realPose.getX() + state.virtualGoal.getX()))
                .getDegrees()
            + 360)
        % 360;
  }
}
