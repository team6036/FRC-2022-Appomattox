package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.config.ShooterConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.util.control.ControllerOutput;
import com.peninsula.frc2022.util.interpolation.InterpolatingDouble;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

  public enum State {
    VELOCITY,
    VISION,
    OFF,
    SHOOT_ON_MOVE
  }

  private final ControllerOutput mMainwheelOutput = new ControllerOutput();
  private final ControllerOutput mHoodwheelOutput = new ControllerOutput();
  private static Shooter sInstance = new Shooter();
  private final MedianFilter mHoodOutputBuffer = new MedianFilter(6);
  private final MedianFilter targetDistFilter = new MedianFilter(10);

  private Shooter() {}

  public static Shooter getInstance() {
    return sInstance;
  }

  public ControllerOutput getShooterOutputs() {
    return mMainwheelOutput;
  }

  public ControllerOutput getHoodOutputs() {
    return mHoodwheelOutput;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    switch (commands.getShooterWanted()) {
      case VELOCITY:
        mMainwheelOutput.setTargetVelocity(
            rpm2Tps(commands.getShooterWantedRPM(), 2048.0), ShooterConstants.flywheelGains);

        mHoodwheelOutput.setTargetVelocity(
            rpm2Tps(commands.getHoodWantedRPM() * 3, 2048.0), ShooterConstants.hoodwheelGains);

        break;
      case VISION:
        mMainwheelOutput.setTargetVelocity(
            rpm2Tps(
                ShooterConstants.flywheelDist2Vel.getInterpolated(
                        new InterpolatingDouble(targetDistFilter.calculate(state.targetDistM)))
                    .value,
                2048),
            ShooterConstants.flywheelGains);
        double hoodOutput =
            mHoodOutputBuffer.calculate(
                rpm2Tps(
                    ShooterConstants.hoodwheelDist2Vel.getInterpolated(
                            new InterpolatingDouble(targetDistFilter.calculate(state.targetDistM)))
                        .value,
                    2048));
        mHoodwheelOutput.setTargetVelocity(hoodOutput, ShooterConstants.hoodwheelGains);
        break;
      case SHOOT_ON_MOVE:
        double robotToVirtualGoalDist =
            new Transform2d(state.realPose, state.virtualGoal)
                .getTranslation()
                .getNorm();
        SmartDashboard.putNumber("distG", robotToVirtualGoalDist);

        mMainwheelOutput.setTargetVelocity(
            rpm2Tps(
                +ShooterConstants.flywheelDist2Vel.getInterpolated(
                        new InterpolatingDouble(robotToVirtualGoalDist))
                    .value,
                2048),
            ShooterConstants.flywheelGains);

        mHoodwheelOutput.setTargetVelocity(
            rpm2Tps(
                ShooterConstants.hoodwheelDist2Vel.getInterpolated(
                        new InterpolatingDouble(robotToVirtualGoalDist))
                    .value,
                2048),
            ShooterConstants.hoodwheelGains);
        break;
      case OFF:
        mMainwheelOutput.setTargetVelocity(rpm2Tps(500, 2048), ShooterConstants.flywheelGains);
        mHoodwheelOutput.setIdle();
        break;
    }
  }

  private double rpm2Tps(double rpm, double tpr) {
    return rpm * tpr / 600.0 * 1.1;
  }
}
