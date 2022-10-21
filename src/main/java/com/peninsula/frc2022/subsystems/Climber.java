package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.controllers.climb.AutoClimbController;
import com.peninsula.frc2022.subsystems.controllers.climb.TeleClimbController;

public class Climber extends SubsystemBase {

  public abstract static class ClimbController {
    protected ClimbOutputs climbOuts;

    public final ClimbOutputs update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
      updateSignal(commands, state);
      return climbOuts;
    }

    public abstract void updateSignal(@ReadOnly Commands commands, @ReadOnly RobotState state);
  }

  public static class ClimbOutputs {
    public double rightEncTicks;
    public double leftEncTicks;

    public ClimbOutputs(double right, double left) {
      this.rightEncTicks = right;
      this.leftEncTicks = left;
    }
  }

  public enum ClimbState {
    AUTO,
    TELE
  }

  public ClimbState mState;

  private ClimbOutputs mOutputs;
  private ClimbController mController;

  private static final Climber sInstance = new Climber();

  public ClimbOutputs getOutputs() {
    return mOutputs;
  }

  private Climber() {}

  public static Climber getInstance() {
    return sInstance;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    ClimbState wantedState = commands.climbWantedState;
    boolean isNewState = mState != wantedState;
    mState = wantedState;
    if (isNewState) {
      switch (mState) {
        case AUTO:
          mController = new AutoClimbController();
          break;
        case TELE:
          mController = new TeleClimbController();
          break;
      }
    }

    if (mController == null) {
      mOutputs =
          new ClimbOutputs(
              AutoClimbController.RightArmState.DOWN.ticks,
              AutoClimbController.LeftArmState.DOWN.ticks);
    } else {
      mOutputs = mController.update(commands, state);
    }

    if (state.rightStatorCurrent > 80.0 && Math.abs(state.rightTickVelo) < 0.5) {
      mOutputs.rightEncTicks += 1500;
    }
    if (state.leftStatorCurrent > 80.0 && Math.abs(state.leftTickVelo) < 0.5) {
      mOutputs.rightEncTicks += 1500;
    }
  }
}
