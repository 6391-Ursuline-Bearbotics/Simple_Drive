// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.UA6391.DifferentialDrive6391;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(
          new WPI_TalonFX(DriveConstants.kLeftMotor1Port),
          new WPI_TalonFX(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(
          new WPI_TalonFX(DriveConstants.kRightMotor1Port),
          new WPI_TalonFX(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive6391 m_drive = new DifferentialDrive6391(m_leftMotors, m_rightMotors);
  
  private LinearSystem<N2, N2, N2> m_driveModel = LinearSystemId.createDrivetrainVelocitySystem(
    DriveConstants.kDriveGearbox,
    DriveConstants.kDriveWeightKg,
    DriveConstants.kWheelDiameterMeters / 2.0,
    DriveConstants.kTrackWidthMeters / 2.0,
    DriveConstants.kDriveMOI,
    DriveConstants.kDriveGearing);

  LinearPlantInversionFeedforward m_driveFeedForward = new LinearPlantInversionFeedforward<>(m_driveModel, 0.02);

  ProfiledPIDController m_AnglePID = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD,
      new TrapezoidProfile.Constraints(5, 10));

  AHRS ahrs;
  try {
    /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    ahrs = new AHRS(SPI.Port.kMXP); 
  } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_drive.setMaxOutput(DriveConstants.kMaxOutputForward, DriveConstants.kMaxOutputRotation);
    m_drive.setMinOutput(DriveConstants.kMinOutputForward, DriveConstants.kMinOutputRotation);
    m_drive.setDeadband(DriveConstants.kDeadbandForward, DriveConstants.kDeadbandRotation);
    m_drive.setRamp(DriveConstants.kRampForward, DriveConstants.kRampRotation);
    m_drive.setDriveStraight(DriveConstants.kDriveStraightLeft, DriveConstants.kDriveStraightRight);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
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

  public void turnToAngle(double targetAngle) {
    double pidout = m_AnglePID.calculate(ahrs.getAngle(), targetAngle);
    Matrix<States, N1> pidMatrix = VecBuilder.fill(pidout, pidout);
    m_driveFeedForward.calculate(pidMatrix);
  }
}
