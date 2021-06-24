// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelConstants;

public class ControlPanel extends SubsystemBase {
  WPI_TalonSRX controlPanel = new WPI_TalonSRX(ControlPanelConstants.controlPanelTalonID);
  DoubleSolenoid controlPanelSolenoid = new DoubleSolenoid(ControlPanelConstants.controlPanelExtendedSolonoidID, ControlPanelConstants.controlPanelRetractedSolonoidID);
  /** Creates a new ControlPanel. */
  public ControlPanel() {
    initTalons(controlPanel);
    SmartDashboard.putString("Control Panel: ", "Retracted");
  }

  public void initTalons(WPI_TalonSRX motor) {
    motor.setSafetyEnabled(true);
    motor.setInverted(true);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor.setSensorPhase(false);
    motor.config_kP(0, (ControlPanelConstants.controlPanelPValue));
    motor.config_kI(0, (ControlPanelConstants.controlPanelIValue));
    motor.config_kD(0, (ControlPanelConstants.controlPanelDValue));
    motor.configMotionCruiseVelocity(ControlPanelConstants.controlPanelVelocity);
		motor.configMotionAcceleration(ControlPanelConstants.controlPanelAcceleration);
    motor.configNominalOutputForward(0);
		motor.configNominalOutputReverse(0);
		motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);
    motor.configAllowableClosedloopError(0, 0, 30);
    motor.setSelectedSensorPosition(0);
  }

  public void controlPanelPercent(double speed) {
    controlPanel.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Control Panel Rotate Position: ", controlPanel.getSelectedSensorPosition());

  }

  public void controlPanelRotate(int position) {
    controlPanel.set(ControlMode.MotionMagic, position);
  }

  public void retractControlPanel() {
    controlPanelSolenoid.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Control Panel", "Retracted");
  }
  

  public void extendControlPanel() {
    controlPanel.setSelectedSensorPosition(0);
    SmartDashboard.putNumber("Control Panel Position: ", controlPanel.getSelectedSensorPosition());
    controlPanelSolenoid.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Control Panel", "Extended");
  }

  public DoubleSolenoid.Value isControlPanelUpOrDown() {
    return controlPanelSolenoid.get();
  }
 
  public boolean isColorWheelAtPos(int position) {
    return (controlPanel.getSelectedSensorPosition() > (position - 100) && 
    (controlPanel.getSelectedSensorPosition() < (position + 100))); 
  }

  public void resetEncoderPosition() {
    controlPanel.setSelectedSensorPosition(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
