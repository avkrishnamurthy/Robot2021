// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.subsystems.ControlPanel;

public class Position extends CommandBase {
  private final ControlPanel m_controlPanel;
  /** Creates a new Position. */
  public Position(ControlPanel controlPanel) {
    this.m_controlPanel = controlPanel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(controlPanel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controlPanel.resetEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controlPanel.controlPanelRotate(ControlPanelConstants.controlPanelNextColorPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanel.controlPanelPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controlPanel.isColorWheelAtPos(ControlPanelConstants.controlPanelNextColorPosition);
  }
}
