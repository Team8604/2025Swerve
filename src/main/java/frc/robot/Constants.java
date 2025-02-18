// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final int kOperatorControllerPort = 0;
  }

  public static class EffectorConstants {
    // TEMPORARY ID
    public static final int kEffector = 50;

    // TEMPORARY speeds
    public static final double kMaxSpeed = 0.5;
    public static final double kIntakeSpeed = 0.5;
  }

  public static class WristConstants {
    // TEMPORARY CAN IDs for
    public static final int kTwist = 51;
    public static final int kTilt = 52;
    public static final int kTiltEncoder = 53;
    public static final int kTwistEncoder = 54;

    // TEMPORARY speeds TEMPORARY PLACEHOLDERS
    public static final double kMaxTwistSpeed = 0.5;
    public static final double kMaxTiltSpeed = 0.5;

    // Wrist Positions TEMPORARY PLACEHOLDERS
    public static final double kStartingPosition = 0;
    public static final double kCoralFirstLevelPosition = 0;

  }
}
