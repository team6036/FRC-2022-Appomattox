package com.peninsula.frc2022.behavior.routines.drive;

import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Swerve;

/**
 * Points drivetrain at a vision target
 */
public class AlignRoutine extends TimeoutRoutineBase {

  public AlignRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.swerveWanted = Swerve.State.ALIGN;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    return false;
  }
}
