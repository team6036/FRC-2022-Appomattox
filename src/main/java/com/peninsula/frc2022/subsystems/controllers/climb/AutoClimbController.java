package com.peninsula.frc2022.subsystems.controllers.climb;

import com.peninsula.frc2022.config.ClimberConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Climber;

public class AutoClimbController extends Climber.ClimbController {
  public enum RightArmState {
    DOWN(ClimberConstants.rightDownTicks),
    FULL_EXTEND(ClimberConstants.rightExtendTicks),
    CLOSE_TO_EXTEND(ClimberConstants.rightPartialTicks),
    STRETCH_MIN(ClimberConstants.rightDownTicks - 6000);

    public double ticks;

    RightArmState(double i) {
      this.ticks = i;
    }
  }

  public enum LeftArmState {
    DOWN(ClimberConstants.leftDownTicks),
    FULL_EXTEND(ClimberConstants.leftExtendTicks),
    CLOSE_TO_EXTEND(ClimberConstants.leftPartialTicks),
    STRETCH_MIN(ClimberConstants.leftDownTicks - 6000);

    public double ticks;

    LeftArmState(double i) {
      this.ticks = i;
    }
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    climbOuts =
        new Climber.ClimbOutputs(
            commands.rightStateClimbWanted.ticks, commands.leftStateClimbWanted.ticks);
  }
}
