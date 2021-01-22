// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.DifferentialDrive6391;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(
          new PWMVictorSPX(DriveConstants.kLeftMotor1Port),
          new PWMVictorSPX(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(
          new PWMVictorSPX(DriveConstants.kRightMotor1Port),
          new PWMVictorSPX(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive6391 m_drive = new DifferentialDrive6391(m_leftMotors, m_rightMotors);

  SlewRateLimiter forwardRamp = new SlewRateLimiter(DriveConstants.kForwardRamp);
  SlewRateLimiter rotationRamp = new SlewRateLimiter(DriveConstants.kRotationRamp);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(forwardRamp.calculate(fwd), rotationRamp.calculate(rot));
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutputForward, double maxOutputRotation) {
    m_drive.setMaxOutput(maxOutputForward, maxOutputRotation);
  }

  public void setDeadband(double deadbandForward, double deadbandRotation) {
    m_drive.setDeadband(deadbandForward, deadbandRotation);
  }
}
