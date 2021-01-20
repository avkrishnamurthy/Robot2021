// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ScoopConstants;

public class Scoop extends SubsystemBase {

  WPI_TalonSRX scoopArm = new WPI_TalonSRX(ScoopConstants.scoopArmTalonID);
  WPI_TalonSRX topIntake =  new WPI_TalonSRX(ScoopConstants.topIntakeTalonID);
  WPI_TalonSRX bottomIntake = new WPI_TalonSRX(ScoopConstants.bottomIntakeTalonID);
  /** Creates a new Scoop. */
  public Scoop() {
    // Configure Scoop Arm Talon
    scoopArm.configFactoryDefault();
    initTalons(scoopArm);
    SmartDashboard.putNumber("Scoop value: ", 0);

    // Configure Intake Talons
    topIntake.configFactoryDefault();
    initIntakeTalons(topIntake, false);
    bottomIntake.configFactoryDefault();
    initIntakeTalons(bottomIntake, true);
    
  }

  // Methods for talon configuration 
  public void initTalons(WPI_TalonSRX motor) {
    motor.setSafetyEnabled(true);
    motor.setInverted(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setSensorPhase(false);
    motor.config_kP(0, ScoopConstants.scoopArmPValue);
    motor.config_kI(0, ScoopConstants.scoopArmIValue);
    motor.config_kD(0, ScoopConstants.scoopArmDValue);
    motor.config_kF(0, 0.9);
    motor.configMotionCruiseVelocity(ScoopConstants.scoopArmVelocity);
    motor.configMotionAcceleration(ScoopConstants.scoopArmAcceleration);
    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);
    motor.configAllowableClosedloopError(0, 0, 30);
    motor.setSelectedSensorPosition(0);
  }

  public void initIntakeTalons(WPI_TalonSRX motor, boolean isBottom) {
    motor.configMotionCruiseVelocity(1500);
    motor.setSensorPhase(false);
    motor.setInverted(isBottom);
  }

  // ----- SCOOP ARM METHODS -----
  // Methods used to move scoop arm to preset locations
  public int horizontalPos = -63500;
  public int gearRatio = 108;
  public double ticksPerDegree = 2048 * gearRatio /360;
  public double maxFeedForward = .099;
  public int currentPos = scoopArm.getSelectedSensorPosition();
  public double degrees = (currentPos - horizontalPos) /ticksPerDegree;
  public double radians = java.lang.Math.toRadians(degrees);
  public double cosScalar = java.lang.Math.cos(radians);

  public void setScoopArmPos(int position) {
    // double cosScalar = Math.cos(Math.toRadians(
        //(scoopArm.getSelectedSensorPosition() - horizontalPos) / ticksPerDegree));
    //scoopArm.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, maxFeedForward * cosScalar);
    scoopArm.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, maxFeedForward*cosScalar);
    //scoopArm.set(ControlMode.MotionMagic, position);
    SmartDashboard.putNumber("Scoop Position after Button:", scoopArm.getSelectedSensorPosition());
    
  }

  public void stopScoopArm() {
    scoopArm.set(ControlMode.PercentOutput, 0.0);
  }


  // Methods used to check position of Scoop Arm
  public boolean isScoopArmAtPos(int position) {
    return ((scoopArm.getSelectedSensorPosition() > (position - ScoopConstants.scoopArmMOE)) && 
      (scoopArm.getSelectedSensorPosition() < (position + ScoopConstants.scoopArmMOE)));
  }

  public void manualScoopArm() {
    scoopArm.set(.15 * RobotContainer.getClimbY());
    int manualCurrentPos = scoopArm.getSelectedSensorPosition();
    double manualDegrees = (manualCurrentPos - horizontalPos) /ticksPerDegree;
    SmartDashboard.putNumber("Scoop Angle: ", manualDegrees);
    SmartDashboard.putNumber("Scoop position: ", scoopArm.getSelectedSensorPosition());
  }

  // ----- SCOOP INTAKE METHODS -----
  public void scoopIntakeCollect() {
    topIntake.set(ControlMode.PercentOutput,  -1);
    bottomIntake.set(ControlMode.PercentOutput,  -1);
  }

  public void scoopIntakeRest() {
    topIntake.set(ControlMode.PercentOutput, 0);
    bottomIntake.set(ControlMode.PercentOutput, 0);
  }

  public void scoopIntakeExpel() {
    topIntake.set(ControlMode.PercentOutput, 1);
    bottomIntake.set(ControlMode.PercentOutput, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
