package com.peninsula.frc2022.subsystems;

import com.ctre.phoenix.led.Animation;
import com.peninsula.frc2022.config.LightingConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import edu.wpi.first.wpilibj.RobotController;
import java.util.*;

public class Lighting extends SubsystemBase {

  public enum State {
    OFF,
    SHOOTING_ON_MOVE_ALIGN,
    ALIGN,
    SHOOTING,
    INTAKING,
    IDLE,
    AUTO
  }

  private static Lighting sInstance = new Lighting();
  private State mState;

  private Lighting() {}

  public static Lighting getInstance() {
    return sInstance;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    State wantedState = commands.lightingWantedState;
    if (RobotController.getBatteryVoltage() < LightingConstants.minVoltageToFunction)
      wantedState = State.OFF;
    boolean isNewState = mState != wantedState;
    mState = wantedState;
    if (isNewState) {
      switch (mState) {
        case SHOOTING_ON_MOVE_ALIGN:
          mOutput = LightingConstants.blueBlink;
          break;
        case ALIGN:
          mOutput = LightingConstants.blueSolid;
          break;
        case SHOOTING:
          mOutput = LightingConstants.greenBlink;
          break;
        case IDLE:
          mOutput = LightingConstants.goldSolid;
          break;
        case AUTO:
          mOutput = LightingConstants.rainbow;
          break;
      }
    }
  }

  Animation mOutput;

  public Animation getOutput() {
    return mOutput;
  }
}
