package com.peninsula.frc2022.subsystems.controllers.climb;

import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Climber;

public class TeleClimbController extends Climber.ClimbController {
  @Override
  public void updateSignal(Commands commands, RobotState state) {
    climbOuts =
        new Climber.ClimbOutputs(commands.rightTicksClimbWanted, commands.leftTicksClimbWanted);
  }
}
