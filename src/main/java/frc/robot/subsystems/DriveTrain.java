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
  WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.driveLeftFrontTalonID);
  WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveConstants.driveLeftBackTalonID);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.driveRightFrontTalonID);
  WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveConstants.driveRightBackTalonID);

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);
  DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);

  DoubleSolenoid gearShift = new DoubleSolenoid(DriveConstants.driveHighGearSolenoidID, DriveConstants.driveLowGearSolenoidID);
  public boolean isHighGear;
  
  public static AHRS nav = new AHRS(SPI.Port.kMXP);

    private final Encoder leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                  DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder rightEncoder =
      new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);

  
  // Lasershark leftShark = new Lasershark(DriveConstants.leftLidar);
  // Lasershark rightShark = new Lasershark(DriveConstants.rightLidar);

  // Test
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  public DriveTrain(){
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    initTalons(leftFront, true, false);
    initTalons(leftBack, true, false);
    initTalons(rightFront, false, false);
    initTalons(rightBack, false, false);
    SmartDashboard.putBoolean("Left Sensor: ", true);
    SmartDashboard.putBoolean("Left Motor: ", false);
    SmartDashboard.putBoolean("Right Sensor: ", false);
    SmartDashboard.putBoolean("Right Motor: ", false);
    resetEncoders();

    gearShift.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Gear: ", "Low Gear");
    isHighGear = false;
    SmartDashboard.putBoolean("High Gear: ", isHighGear);
    // SmartDashboard.putNumber("Left Distance: ", 0);
    // SmartDashboard.putNumber("Right Distance: ", 0);

    SmartDashboard.putNumber("Left encoder: ", 0);
    SmartDashboard.putNumber("Right encoder: ", 0);
    
    // Test
    nav.reset();
    nav.resetDisplacement();
    SmartDashboard.putNumber("Gyro: ", -nav.getYaw());
    SmartDashboard.putNumber("Nav x: ", 0);
    SmartDashboard.putNumber("Nav y: ", 0);
    SmartDashboard.putNumber("Nav z: ", 0);
    
    odometry.resetPosition(new Pose2d(), getHeading());
    SmartDashboard.putNumber("X pose", 0);
    SmartDashboard.putNumber("Y pose", 0);
  }
  
  /**
   * Initialize Motor Controllers
   * @param motor motor name
   * @param sensorPhase reverse polarity of the sensor
   * @param invert invert direction of motor
   */
  public void initTalons(WPI_TalonSRX motor, boolean sensorPhase, boolean invert) {
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
    // SmartDashboard.putNumber("Left Distance: ", leftShark.getDistanceMeters());
    // SmartDashboard.putNumber("Right Distance: ", rightShark.getDistanceMeters());

    SmartDashboard.putNumber("Left encoder: ", leftFront.getSelectedSensorPosition()); 
    SmartDashboard.putNumber("Right encoder: ", rightFront.getSelectedSensorPosition());

    // SmartDashboard.putNumber("Gyro: ", -nav.getYaw());
    SmartDashboard.putNumber("Nav x: ", nav.getDisplacementX());
    SmartDashboard.putNumber("Nav y: ", nav.getDisplacementY());
    SmartDashboard.putNumber("Nav z: ", nav.getDisplacementZ());

    odometry.update(getHeading(), 
      Units.inchesToMeters(6*Math.PI) * leftFront.getSelectedSensorPosition() / 
          (2048 * (isHighGear? 6.6 : 15.0)), 
      Units.inchesToMeters(6*Math.PI) * rightFront.getSelectedSensorPosition() / 
          (2048 * (isHighGear? 6.6 : 15.0)));
    // odometry.update(getHeading(), 
      // Units.inchesToMeters(6*Math.PI) * leftFront.getSelectedSensorPosition() / 33000, 
      // Units.inchesToMeters(6*Math.PI) * rightFront.getSelectedSensorPosition() / 33000);
    SmartDashboard.putNumber("X pose", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y pose", odometry.getPoseMeters().getTranslation().getY());

    SmartDashboard.putNumber("left encoder get: ", leftEncoder.get());
    SmartDashboard.putNumber("right encoder get: ", rightEncoder.get());
    SmartDashboard.putNumber("left encoder position: ", leftEncoder.getDistance());
    SmartDashboard.putNumber("right encoder position: ", rightEncoder.getDistance());
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
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(-rightVolts);
    diffDrive.feed();
  }

  public Rotation2d getHeading() {
    // return Rotation2d.fromDegrees(-Math.IEEEremainder(nav.getAngle(), 360));
    return Rotation2d.fromDegrees(-nav.getYaw());
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    //return new DifferentialDriveWheelSpeeds(leftFront.getSelectedSensorVelocity(), rightFront.getSelectedSensorVelocity());
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
    return nav.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
    
  }

    /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;    
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
    odometry.update(getHeading(), leftEncoder.getDistance(),
                      rightEncoder.getDistance());
  }

//UPDATED CODE ---------



//    WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.driveLeftFrontTalonID);
//    WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveConstants.driveLeftBackTalonID);
//    WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.driveRightFrontTalonID);
//    WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveConstants.driveRightFrontTalonID);

//   SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
//   SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFront, rightBack);
//   DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);

//   DoubleSolenoid gearShift = new DoubleSolenoid(DriveConstants.driveHighGearSolenoidID, DriveConstants.driveLowGearSolenoidID);
//   public boolean isHighGear;

//   private final Encoder leftEncoder =
//       new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
//                   DriveConstants.kLeftEncoderReversed);

// // The right-side drive encoder
//   private final Encoder rightEncoder =
//       new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
//                   DriveConstants.kRightEncoderReversed);

//   private final AHRS gyro = new AHRS(SPI.Port.kMXP);

//   // Odometry class for tracking robot pose
//   private final DifferentialDriveOdometry odometry;

//   public DriveTrain() {
//     // Sets the distance per pulse for the encoders
//     // leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
//     // rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

//     resetEncoders();
//     odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
//     gyro.reset();
//     gyro.resetDisplacement();
//     initTalons(leftFront, true, false);
//     initTalons(leftBack, true, false);
//     initTalons(rightFront, false, false);
//     initTalons(rightBack, false, false);
//     SmartDashboard.putBoolean("Left Sensor: ", true);
//     SmartDashboard.putBoolean("Left Motor: ", false);
//     SmartDashboard.putBoolean("Right Sensor: ", false);
//     SmartDashboard.putBoolean("Right Motor: ", false);

//     gearShift.set(DoubleSolenoid.Value.kReverse);
//     SmartDashboard.putString("Gear: ", "Low Gear");
//     isHighGear = false;
//     SmartDashboard.putBoolean("High Gear: ", isHighGear);

//     SmartDashboard.putNumber("Left encoder: ", 0);
//     SmartDashboard.putNumber("Right encoder: ", 0);
    
//     // Test
//     // gyro.reset();
//     // gyro.resetDisplacement();
//     // SmartDashboard.putNumber("Gyro: ", -gyro.getYaw());
//     // SmartDashboard.putNumber("Nav x: ", 0);
//     // SmartDashboard.putNumber("Nav y: ", 0);
//     // SmartDashboard.putNumber("Nav z: ", 0);
    
//     // odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
//     // SmartDashboard.putNumber("X pose", 0);
//     // SmartDashboard.putNumber("Y pose", 0);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(),
//                       rightEncoder.getDistance());
//   }

//   /**
//    * Initialize Motor Controllers
//    * @param motor motor name
//    * @param sensorPhase reverse polarity of the sensor
//    * @param invert invert direction of motor
//    */
//   public void initTalons(WPI_TalonSRX motor, boolean sensorPhase, boolean invert) {
//     motor.configFactoryDefault();
//     motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
//     motor.setSensorPhase(sensorPhase);
//     motor.setInverted(invert);
//     motor.setSelectedSensorPosition(0);
//   }

//   public void reInitTalons() {
//     boolean lSensor = SmartDashboard.getBoolean("Left Sensor: ", true);
//     boolean lMotor = SmartDashboard.getBoolean("Left Motor: ", false);
//     boolean rSensor = SmartDashboard.getBoolean("Right Sensor: ", false);
//     boolean rMotor = SmartDashboard.getBoolean("Right Motor: ", false);
//     initTalons(leftFront, lSensor, lMotor);
//     initTalons(leftBack, lSensor, lMotor);
//     initTalons(rightFront, rSensor, rMotor);
//     initTalons(rightBack, rSensor, rMotor);
//   }

//   public void curvatureDrive(double xSpeed, double zRotation, Boolean isQuickTurn) {
//     diffDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
//     // SmartDashboard.putNumber("Left Distance: ", leftShark.getDistanceMeters());
//     // SmartDashboard.putNumber("Right Distance: ", rightShark.getDistanceMeters());

//     SmartDashboard.putNumber("Left encoder: ", leftFront.getSelectedSensorPosition()); 
//     SmartDashboard.putNumber("Right encoder: ", rightFront.getSelectedSensorPosition());

//     // SmartDashboard.putNumber("Gyro: ", -gyro.getYaw());
//     SmartDashboard.putNumber("gyro x: ", gyro.getDisplacementX());
//     SmartDashboard.putNumber("gyro y: ", gyro.getDisplacementY());
//     SmartDashboard.putNumber("gyro z: ", gyro.getDisplacementZ());
//     SmartDashboard.putNumber("Gyro Heading: ", gyro.getAngle());
//     odometry.update(Rotation2d.fromDegrees(getHeading()), 
//       Units.inchesToMeters(6*Math.PI) * leftFront.getSelectedSensorPosition() / 
//           (2048 * (isHighGear? 6.6 : 15.0)), 
//       Units.inchesToMeters(6*Math.PI) * rightFront.getSelectedSensorPosition() / 
//           (2048 * (isHighGear? 6.6 : 15.0)));
//     // odometry.update(getHeading(), 
//       // Units.inchesToMeters(6*Math.PI) * leftFront.getSelectedSensorPosition() / 33000, 
//       // Units.inchesToMeters(6*Math.PI) * rightFront.getSelectedSensorPosition() / 33000);
//     SmartDashboard.putNumber("X pose", odometry.getPoseMeters().getTranslation().getX());
//     SmartDashboard.putNumber("Y pose", odometry.getPoseMeters().getTranslation().getY());
//   }

//   public void shiftToHighGear() {
//     gearShift.set(DoubleSolenoid.Value.kForward);
//     SmartDashboard.putString("Gear: ", "High Gear");
//     isHighGear = true;
//     SmartDashboard.putBoolean("High Gear: ", isHighGear);
//   }

//   public void shiftToLowGear() {
//     gearShift.set(DoubleSolenoid.Value.kReverse);
//     SmartDashboard.putString("Gear: ", "Low Gear");
//     isHighGear = false;
//     SmartDashboard.putBoolean("High Gear: ", isHighGear);
//   }

  // /**
  //  * Returns the current wheel speeds of the robot.
  //  *
  //  * @return The current wheel speeds.
  //  */
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  // }

//   /**
//    * Returns the currently-estimated pose of the robot.
//    *
//    * @return The pose.
//    */
//   public Pose2d getPose() {
//     return odometry.getPoseMeters();
//   }

//    /**
//    * Zeroes the heading of the robot.
//    */
//   public void zeroHeading() {
//     gyro.reset();
//   }

//   /**
//    * Resets the odometry to the specified pose.
//    *
//    * @param pose The pose to which to set the odometry.
//    */
//   public void resetOdometry(Pose2d pose) {
//     resetEncoders();
//     odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
//   }


//   // public Rotation2d getHeading() {
//   //   // return Rotation2d.fromDegrees(-Math.IEEEremainder(gyro.getAngle(), 360));
//   //   return Rotation2d.fromDegrees(-gyro.getYaw());
//   // }

//     /**
//    * Returns the heading of the robot.
//    *
//    * @return the robot's heading in degrees, from -180 to 180
//    */
//   public double getHeading() {
//     return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
//     // return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
//   }

  //  /**
  //  * Returns the turn rate of the robot.
  //  *
  //  * @return The turn rate of the robot, in degrees per second
  //  */
  // public double getTurnRate() {
  //   return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  // /**
  //  * Resets the drive encoders to currently read a position of 0.
  //  */
  // public void resetEncoders() {
  //   leftEncoder.reset();
  //   rightEncoder.reset();
  // }

  //   /**
  //  * Gets the left drive encoder.
  //  *
  //  * @return the left drive encoder
  //  */
  // public Encoder getLeftEncoder() {
  //   return leftEncoder;
  // }

  // /**
  //  * Gets the right drive encoder.
  //  *
  //  * @return the right drive encoder
  //  */
  // public Encoder getRightEncoder() {
  //   return rightEncoder;
  // }

  // /**
  //  * Gets the average distance of the two encoders.
  //  *
  //  * @return the average of the two encoder readings
  //  */
  // public double getAverageEncoderDistance() {
  //   return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  // }

  // /**
  //  * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
  //  *
  //  * @param maxOutput the maximum output to which the drive will be constrained
  //  */
  //      public void setMaxOutput(double maxOutput) {
  //      diffDrive.setMaxOutput(maxOutput);
  //  }

}
