/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.util.Units;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

   //WPI_TalonFX
  WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.driveLeftFrontTalonID);
  WPI_TalonFX leftBack = new WPI_TalonFX(DriveConstants.driveLeftBackTalonID);
  WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.driveRightFrontTalonID);
  WPI_TalonFX rightBack = new WPI_TalonFX(DriveConstants.driveRightBackTalonID);

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);

  DoubleSolenoid gearShift = new DoubleSolenoid(DriveConstants.driveHighGearSolenoidID, DriveConstants.driveLowGearSolenoidID);
  public boolean isHighGear;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(9.090140961492919);
  private final DifferentialDriveOdometry odometry;
  Pose2d pose;
  public static AHRS nav = new AHRS(SPI.Port.kMXP);

    private final Encoder leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                  DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder rightEncoder =
      new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);

  
  // Test
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  public DriveTrain(){
    nav.reset();
    initTalons(leftFront, true, false);
    initTalons(leftBack, true, false);
    initTalons(rightFront, false, false);
    initTalons(rightBack, false, false);
    resetEncoders();
    gearShift.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Gear: ", "Low Gear");
    isHighGear = false;
    SmartDashboard.putBoolean("High Gear: ", isHighGear);
    odometry = new DifferentialDriveOdometry(getHeading());

    
    // // Test
    // nav.reset();
    // nav.resetDisplacement();
    // SmartDashboard.putNumber("Gyro: ", -nav.getYaw());
    // SmartDashboard.putNumber("Nav x: ", 0);
    // SmartDashboard.putNumber("Nav y: ", 0);
    // SmartDashboard.putNumber("Nav z: ", 0);
    
    // odometry.resetPosition(new Pose2d(), getHeading());
    // SmartDashboard.putNumber("X pose", 0);
    // SmartDashboard.putNumber("Y pose", 0);
  }
  
  /**
   * Initialize Motor Controllers
   * @param motor motor name
   * @param sensorPhase reverse polarity of the sensor
   * @param invert invert direction of motor
   */
  public void initTalons(WPI_TalonFX motor, boolean sensorPhase, boolean invert) {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSensorPhase(sensorPhase);
    motor.setInverted(invert);
    motor.setSelectedSensorPosition(0);
  }

  // Temp
  public void reInitTalons() {
    boolean lSensor = SmartDashboard.getBoolean("Left Sensor: ", true);
    boolean lMotor = SmartDashboard.getBoolean("Left Motor: ", false);
    boolean rSensor = SmartDashboard.getBoolean("Right Sensor: ", false);
    boolean rMotor = SmartDashboard.getBoolean("Right Motor: ", false);
    initTalons(leftFront, lSensor, lMotor);
    initTalons(leftBack, lSensor, lMotor);
    initTalons(rightFront, rSensor, rMotor);
    initTalons(rightBack, rSensor, rMotor);
  }

  public boolean isLeftAbovePos(int position) {
    return ((leftFront.getSelectedSensorPosition() <= position));
  }

  public boolean isRightAbovePos(int position) {
    return ((rightFront.getSelectedSensorPosition() > (position - DriveConstants.driveEncoderMOE)) && 
      (rightFront.getSelectedSensorPosition() < (position + DriveConstants.driveEncoderMOE)));
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn){
    diffDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);

    SmartDashboard.putNumber("Left Distance: ", getLeftEncoder()); 
    SmartDashboard.putNumber("Right Distance: ", getRightEncoder());
    SmartDashboard.putNumber("Heading", -nav.getAngle());

    odometry.update(getHeading(), 
      Units.inchesToMeters(6*Math.PI) * leftFront.getSelectedSensorPosition() / 
          (2048 * (isHighGear? 6.6 : 15.0)), 
      Units.inchesToMeters(6*Math.PI) * rightFront.getSelectedSensorPosition() / 
          (2048 * (isHighGear? 6.6 : 15.0)));

  }

  public void shiftToHighGear() {
    gearShift.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Gear: ", "High Gear");
    isHighGear = true;
    SmartDashboard.putBoolean("High Gear: ", isHighGear);
  }

  public void shiftToLowGear() {
    gearShift.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Gear: ", "Low Gear");
    isHighGear = false;
    SmartDashboard.putBoolean("High Gear: ", isHighGear);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(-leftVolts);
    rightDrive.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public Rotation2d getHeading() {
    // return Rotation2d.fromDegrees(-Math.IEEEremainder(nav.getAngle(), 360));
    //return Rotation2d.fromDegrees(-nav.getYaw());
    return Rotation2d.fromDegrees(-nav.getAngle());
    //return Math.IEEEremainder(nav.getAngle(), 360);
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    return new DifferentialDriveWheelSpeeds(leftFront.getSelectedSensorVelocity() / 8.333 * 0.1524 * Math.PI / 60,
                                            rightFront.getSelectedSensorVelocity() / 8.333 * 0.1524 * Math.PI / 60);
    

  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

   /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    nav.reset();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

   /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return -nav.getRate();//nav.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return -nav.getRate();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    
  }

    /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return ((leftFront.getSelectedSensorPosition() * 0.1524 * Math.PI) /(2048.0 * 15.0));
  }

  /**
   * Gets the right drive encoder.w
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    //return ((rightFront.getSelectedSensorPosition() * 0.1524 * Math.PI) /(2048.0 * 15.0));
    return rightFront.getSelectedSensorPosition();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    //return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;    
    return (((leftFront.getSelectedSensorPosition() * 0.1524 * Math.PI) /(2048.0 * 8.333))
             + ((rightFront.getSelectedSensorPosition() * 0.1524 * Math.PI) /(2048.0 * 8.333))) / 2.0;
  }

    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
}

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(
      getHeading(), (leftFront.getSelectedSensorPosition() * 0.1524 * Math.PI) /(2048.0 * 8.333), (rightFront.getSelectedSensorPosition() * 0.1524 * Math.PI) / (2048.0 * 8.333));
      
  }
}