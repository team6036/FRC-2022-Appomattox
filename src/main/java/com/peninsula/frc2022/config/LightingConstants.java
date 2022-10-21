package com.peninsula.frc2022.config;

import com.ctre.phoenix.led.*;

public class LightingConstants {

  public static int minVoltageToFunction = 7;
  public static final Animation greenBlink = new StrobeAnimation(0, 255, 0, 0, 0.1, 40);
  public static final Animation blueBlink = new StrobeAnimation(0, 0, 255, 0, 0.1, 40);
  public static final Animation blueSolid = new StrobeAnimation(0, 0, 255, 0, 0.5, 40);
  public static final Animation goldSolid = new StrobeAnimation(179, 99, 19, 0, 0.5, 40);
  public static final Animation rainbow = new RainbowAnimation(1.0, 0.9, 40);
}
