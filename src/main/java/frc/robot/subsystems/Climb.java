// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  WPI_TalonSRX elevator = new WPI_TalonSRX(ClimbConstants.climbElevatorTalonID);
  WPI_TalonSRX winch = new WPI_TalonSRX(ClimbConstants.climbWinchTalonID);
  Solenoid winchLock = new Solenoid(ClimbConstants.climbWinchLockSolenoidID);

  /** Creates a new Climb. */
  public Climb() {
    initTalons(elevator, true);
    initTalons(winch, false);
    SmartDashboard.putNumber("Elevator position:", 0);
    SmartDashboard.putNumber("Winch position:", 0);
  }

  public void initTalons(WPI_TalonSRX motor, boolean isElevator) {
    motor.configFactoryDefault();
   //motor.setSafetyEnabled(true);
    motor.setInverted(isElevator);
    motor.configSelectedFeedbackSensor(isElevator ? FeedbackDevice.CTRE_MagEncoder_Relative : FeedbackDevice.IntegratedSensor);
    motor.setSensorPhase(false);
    motor.config_kP(0, (isElevator ? ClimbConstants.climbElevatorPValue : ClimbConstants.climbWinchPValue));
    motor.config_kI(0, (isElevator ? ClimbConstants.climbElevatorIValue : ClimbConstants.climbWinchIValue));
    motor.config_kD(0, (isElevator ? ClimbConstants.climbElevatorDValue : ClimbConstants.climbWinchDValue));
    motor.configMotionCruiseVelocity(isElevator ? ClimbConstants.climbElevatorVelocity : ClimbConstants.climbWinchVelocity);
    motor.configMotionAcceleration(isElevator ? ClimbConstants.climbElevatorAcceleration : ClimbConstants.climbWinchAcceleration);
    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);
    motor.configAllowableClosedloopError(0, 0, 100);
    motor.setSelectedSensorPosition(0);
  }

  public void manualElevator() {
    if (isElevatorBelowMaxHeight()) {
      elevator.set(.5 * RobotContainer.getClimbY());
      SmartDashboard.putNumber("Elevator Position: ", elevator.getSelectedSensorPosition());
    }
    else if (RobotContainer.getClimbY() < 0) {
      elevator.set(.5*RobotContainer.getClimbY());
    }

    else {
      elevator.set(0 * RobotContainer.getClimbY());
    }
    //elevator.set(0.5 * RobotContainer.getClimbY());
  }

  public void manualWinch() {
    if (isWinchBelowMaxHeight()) {
      winch.set(.5 * RobotContainer.getClimbY() * ClimbConstants.winchMultiplierConstant);
      SmartDashboard.putNumber("Winch Position: ", winch.getSelectedSensorPosition());
    }
    else if (RobotContainer.getClimbY() < 0) {
      winch.set(.5*RobotContainer.getClimbY() * ClimbConstants.winchMultiplierConstant);
    }

    else {
      winch.set(0 * RobotContainer.getClimbY() * ClimbConstants.winchMultiplierConstant);
    }
    //winch.set(0.5 *RobotContainer.getClimbY());
  }

  public void setElevatorPos(int position) {
    elevator.set(ControlMode.MotionMagic, position);
  }
  
  public void setWinchPos(int position) {
    winch.set(ControlMode.MotionMagic, position);
  }

  public void setClimb(int elevatorPosition, int winchPosition) {
    elevator.set(ControlMode.MotionMagic, elevatorPosition);
    winch.set(ControlMode.MotionMagic, winchPosition);
  }

  
  public boolean isElevatorAtPos(int position) {
    return ((elevator.getSelectedSensorPosition() > (position - ClimbConstants.climbElevatorMOE)) && 
      (elevator.getSelectedSensorPosition() < (position + ClimbConstants.climbElevatorMOE)));
  }

  public boolean isWinchAtPos(int position) {
    return ((winch.getSelectedSensorPosition() > (position - ClimbConstants.climbWinchMOE)) && 
      (winch.getSelectedSensorPosition() < (position + ClimbConstants.climbWinchMOE)));
  }

  public boolean isElevatorBelowMaxHeight() {
    return (elevator.getSelectedSensorPosition() <= ClimbConstants.climbElevatorMaxHeight);
  }

  public boolean isWinchBelowMaxHeight() {
    return (winch.getSelectedSensorPosition() <= ClimbConstants.climbWinchMaxHeight);
  }

  public void stopWinch() {
    winch.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopElevator() {
    elevator.set(ControlMode.PercentOutput, 0.0);
  }

  // ----- Winch Lock -----
  public void lockWinch() {
    winchLock.set(false);
  }

  public void unlockWinch() {
    winchLock.set(true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
