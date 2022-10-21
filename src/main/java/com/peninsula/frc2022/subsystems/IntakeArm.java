package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.config.IntakeConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.util.control.ControllerOutput;

public class IntakeArm extends SubsystemBase {

  public enum State {
    UP,
    DOWN
  }

  private ControllerOutput mOutputs1 = new ControllerOutput();
  private ControllerOutput mOutputs2 = new ControllerOutput();
  private static IntakeArm sInstance = new IntakeArm();

  public double tickPos = 0;

  private IntakeArm() {}

  public static IntakeArm getInstance() {
    return sInstance;
  }

  public ControllerOutput getOutputs1() {
    return mOutputs1;
  }

  public ControllerOutput getOutputs2() {
    return mOutputs2;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {

    switch (commands.intakeArmWanted) {
      case UP:
        mOutputs1.setTargetPosition(IntakeConstants.armUpPos, IntakeConstants.intakeArmGainsHard);
        mOutputs2.setTargetPosition(IntakeConstants.armUpPos, IntakeConstants.intakeArmGainsLight);
        tickPos = IntakeConstants.armUpPos;
        break;
      case DOWN:
        mOutputs1.setTargetPosition(IntakeConstants.armDownPos, IntakeConstants.intakeArmGainsHard);
        mOutputs2.setTargetPosition(
            IntakeConstants.armDownPos, IntakeConstants.intakeArmGainsLight);
        tickPos = IntakeConstants.armDownPos;
        break;
    }
  }
}
