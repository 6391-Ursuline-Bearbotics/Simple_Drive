// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.UA6391.DifferentialDrive6391;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
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

  // These represent our regular encoder objects, which we would
  // create to use on a real robot.
  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  
  private LinearSystem<N2, N2, N2> m_driveModel = LinearSystemId.createDrivetrainVelocitySystem(
    DriveConstants.kDriveGearbox,
    DriveConstants.kDriveWeightKg,
    DriveConstants.kWheelDiameterMeters / 2.0,
    DriveConstants.kTrackWidthMeters / 2.0,
    DriveConstants.kDriveMOI,
    DriveConstants.kDriveGearing);

  LinearPlantInversionFeedforward m_driveFeedForward = new LinearPlantInversionFeedforward<>(m_driveModel, 0.02);

  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DriveConstants.kDriveGearbox,
      DriveConstants.kDriveGearing,
      DriveConstants.kDriveMOI,
      DriveConstants.kDriveWeightKg,
      DriveConstants.kWheelDiameterMeters / 2.0,
      DriveConstants.kTrackWidthMeters,  null);

  ProfiledPIDController m_AnglePID = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD,
      new TrapezoidProfile.Constraints(5, 10));

  private AHRS m_gyro;

  private Field2d m_field = new Field2d();
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_drive.setMaxOutput(DriveConstants.kMaxOutputForward, DriveConstants.kMaxOutputRotation);
    m_drive.setMinOutput(DriveConstants.kMinOutputForward, DriveConstants.kMinOutputRotation);
    m_drive.setDeadband(DriveConstants.kDeadbandForward, DriveConstants.kDeadbandRotation);
    m_drive.setRamp(DriveConstants.kRampForward, DriveConstants.kRampRotation);
    m_drive.setDriveStraight(DriveConstants.kDriveStraightLeft, DriveConstants.kDriveStraightRight);

    if (RobotBase.isSimulation()) { // If our robot is simulated
      m_drive.setRightSideInverted(false);
    }

    m_leftEncoder.setDistancePerPulse(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kEncoderResolution);

    m_AnglePID.enableContinuousInput(-180, 180);
    m_AnglePID.setTolerance(DriveConstants.kPositionTolerance, DriveConstants.kVelocityTolerance);

    m_field = new Field2d();

    SmartDashboard.putData("Field", m_field);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(m_leftMotors.get() * RobotController.getInputVoltage(),
                       m_rightMotors.get() * RobotController.getInputVoltage());

    m_driveSim.update(0.02);

    // From NavX example
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-m_driveSim.getHeading().getDegrees(), 360));

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
  }

  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(),
                      m_leftEncoder.getDistance(),
                      m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("Heading", getHeading());
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

  public double getHeading() {
    double heading = -m_gyro.getYaw();
    if (heading > 180 || heading < 180) {
      heading = Math.IEEEremainder(heading, 360);
    }
    return heading;
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
    double pidout = m_AnglePID.calculate(getHeading(), targetAngle);
    var pidMatrix = VecBuilder.fill(pidout, pidout);
    m_drive.arcadeDrive(0, m_driveFeedForward.calculate(pidMatrix).get(0, 0));
  }
}
