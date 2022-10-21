package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.config.IntakeConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.util.Util;
import com.peninsula.frc2022.util.control.ControllerOutput;
import com.peninsula.frc2022.util.interpolation.InterpolatingDouble;

public class Intake extends SubsystemBase {

  public enum State {
    RUN,
    UNINDEX,
    OFF
  }

  private ControllerOutput mOutputs = new ControllerOutput();
  private static Intake sInstance = new Intake();

  private Intake() {}

  public static Intake getInstance() {
    return sInstance;
  }

  public ControllerOutput getOutputs() {
    return mOutputs;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    switch (commands.intakeWanted) {
      case RUN:
        mOutputs.setPercentOutput(
            Util.clamp(
                IntakeConstants.dtVel2IntakePo.getInterpolated(
                        new InterpolatingDouble(state.avgModuleVel))
                    .value,
                0,
                IntakeConstants.maxRollerPo));
        break;
      case UNINDEX:
        mOutputs.setPercentOutput(IntakeConstants.ejectRollerPo);
        break;
      case OFF:
        mOutputs.setIdle();
        break;
    }
  }
}
