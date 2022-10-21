package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.Robot;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.controllers.drive.*;
import com.peninsula.frc2022.util.control.SwerveOutputs;

/** Represents the drivetrain. Uses {@link #mController} to generate {@link #mOutputs}. */
public class Swerve extends SubsystemBase {

  public enum State {
    NEUTRAL,
    TELEOP,
    ALIGN,
    AUTO,
    SHOOT_ON_MOVE
  }

  public abstract static class SwerveController {

    protected SwerveOutputs mOutputs = new SwerveOutputs();
    protected boolean mRumbleOutput = false;

    public final SwerveOutputs update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
      updateSignal(commands, state);
      return mOutputs;
    }

    /** Should set {@link #mOutputs} to reflect what is currently wanted by {@link Commands}. */
    public abstract void updateSignal(@ReadOnly Commands commands, @ReadOnly RobotState state);
  }

  private static Swerve sInstance = new Swerve();
  private SwerveController mController;
  private State mState;
  private SwerveOutputs mOutputs = new SwerveOutputs();
  private boolean mRumbleOutput = false;
  private boolean mZeroReq = false;

  public boolean getZero() {
    return mZeroReq;
  }

  private Swerve() {}

  public static Swerve getInstance() {
    return sInstance;
  }

  public SwerveOutputs getOutputs() {
    return mOutputs;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    State wantedState = commands.swerveWanted;
    boolean isNewState = mState != wantedState;
    mState = wantedState;
    if (isNewState) {
      switch (wantedState) {
        case NEUTRAL:
          mController = null;
          break;
        case TELEOP:
          mController = new TeleopController();
          break;
        case AUTO:
          mController = new AutoDriveController();
          break;
        case ALIGN:
          if (!Robot.isReal()) mController = null; // Simulation
          else mController = new AlignController();
          break;
        case SHOOT_ON_MOVE:
          mController = new ShootOnMoveController();
          break;
      }
    }

    if (mController == null) {
      mOutputs.setIdle(true);
    } else {
      mOutputs = mController.update(commands, state);
      mRumbleOutput = mController.mRumbleOutput;
    }
  }
}
