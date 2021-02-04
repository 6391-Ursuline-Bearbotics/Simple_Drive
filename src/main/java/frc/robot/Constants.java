// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    public static final double kForwardMaxOutput = 1;
    public static final double kRotationMaxOutput = 0.75;

    public static final double kPartialPower = 0.5;

    public static final double kForwardDeadband = 0.05;
    public static final double kRotationDeadband = 0.2;

    public static final double kForwardRamp = 0.2;
    public static final double kRotationRamp = 0.2;

    // Differential Drive setup parameters.  These control how the drivers input translates to motor power.
    public static final double kMaxOutputForward = 1; // % motor ouput
    public static final double kMaxOutputRotation = 0.6; // % motor ouput
    public static final double kDeadbandForward = 0.05; // % motor ouput
    public static final double kDeadbandRotation = 0.05; // % motor ouput
    public static final double kRampForward = 0.2; // Seconds to go from min to max motor %
    public static final double kRampRotation = 0.2; // Seconds to go from min to max motor %
    public static final double kMinOutputForward = 0; // Minimum % forward power
    public static final double kMinOutputRotation = 0.2; // Minimum % rotation power
    public static final double kDriveStraightLeft = 1; // Multiplier on motor power to help drive straight
    public static final double kDriveStraightRight = 0.98; // Multiplier on motor power to help drive straight
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
