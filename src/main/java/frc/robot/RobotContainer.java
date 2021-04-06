/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoopConstants;
import frc.robot.commands.DriveForward;
import frc.robot.commands.GalacticSearchPathB;
import frc.robot.commands.RedPathAGalacticSearch;
import frc.robot.commands.RedPathAGalaticSearchFull;
import frc.robot.commands.AutoShootTrench;
import frc.robot.commands.climb.LevelClimb;
import frc.robot.commands.climb.LiftOff;
import frc.robot.commands.climb.ManualClimb;
import frc.robot.commands.climb.ManualElevator;
import frc.robot.commands.climb.ManualWinch;
import frc.robot.commands.climb.PresetClimb;
import frc.robot.commands.controlPanel.Activate;
import frc.robot.commands.controlPanel.Position;
import frc.robot.commands.controlPanel.Rotation;
import frc.robot.commands.driveTrain.AutoBarrel;
import frc.robot.commands.driveTrain.AutoBounce;
import frc.robot.commands.driveTrain.AutoSlalom;
import frc.robot.commands.driveTrain.Circle;
import frc.robot.commands.driveTrain.CurvatureDrive;
import frc.robot.commands.driveTrain.HighGear;
import frc.robot.commands.driveTrain.LowGear;
import frc.robot.commands.driveTrain.RedPathAFirstBall;
import frc.robot.commands.driveTrain.StartToLowGoal;
import frc.robot.commands.driveTrain.StopDrive;
import frc.robot.commands.scoop.ScoopExpel;
import frc.robot.commands.scoop.ScoopIntake;
import frc.robot.commands.scoop.ScoopIntakePos;
import frc.robot.commands.scoop.ScoopLoadingStationPos;
import frc.robot.commands.scoop.ScoopLowGoalPos;
import frc.robot.commands.scoop.ScoopManual;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Scoop;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrain driveTrain;
  private final Climb climb;
  private final Scoop scoop;
  private final ControlPanel controlPanel;
  private final Command simpleAuto;
  private final Command complexAuto;
  private final Command galacticSearchPathA;
  private final Command galacticSearchPathB;
  private final Command autoSlalom;
  private final Command autoBounce;
  private final Command autoBarrel;
  private final RamseteCommand ramseteCommand;
  static Joystick buttonStick;
  static Joystick throttleStick;
  static Joystick curveStick;
  JoystickButton isQuickTurnButton;
  //SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      driveTrain=new DriveTrain();
      climb=new Climb();
      scoop=new Scoop();
      controlPanel=new ControlPanel();
      simpleAuto=new StartToLowGoal(driveTrain);
      complexAuto = new AutoShootTrench(driveTrain, scoop);
      galacticSearchPathA = new RedPathAGalaticSearchFull(driveTrain, scoop);
      galacticSearchPathB = new GalacticSearchPathB(driveTrain, scoop);
      autoSlalom = new AutoSlalom(driveTrain);
      autoBounce = new AutoBounce(driveTrain);
      autoBarrel = new AutoBarrel(driveTrain);

      buttonStick=new Joystick(OIConstants.buttonJoystickID);
      throttleStick=new Joystick(OIConstants.throttleJoystickID);
      curveStick=new Joystick(OIConstants.curveJoystickID);
      isQuickTurnButton=new JoystickButton(curveStick,OIConstants.isQuickTurnButtonID);

      driveTrain.setDefaultCommand(new CurvatureDrive(
      driveTrain, 
      () -> throttleStick.getY(GenericHID.Hand.kLeft),
      () -> -curveStick.getX(GenericHID.Hand.kRight), 
      () ->isQuickTurnButton.get()));
      
      configureButtonBindings();
      


        // var autoVoltageConstraint = 
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(DriveConstants.ksVolts, 
    //                                    DriveConstants.kvVoltSecondsPerMeter,
    //                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //     10);

    // TrajectoryConfig config = 
    //     new TrajectoryConfig(
    //             AutonomousConstants.kMaxSpeedMetersPerSecond,
    //             AutonomousConstants.kMaxAccelerationMetersPerSecondSquared)
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         .addConstraint(autoVoltageConstraint);
    
            String trajectoryJSON = "paths/StartToLowGoal.wpilib.json";
            //String trajectoryJSON = "paths/LowGoalToTrench.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            ramseteCommand = new RamseteCommand(
                trajectory,
                driveTrain::getPose,
                new RamseteController(AutonomousConstants.kRamseteB, AutonomousConstants.kRamseteZeta),
                new SimpleMotorFeedforward(0.617,
                                           0.221,
                                           0.015),
                DriveConstants.kDriveKinematics,
                driveTrain::getWheelSpeeds, //adjusted getWheelSpeeds first
                new PIDController(0.0000000000017, 0, 0),
                new PIDController(0.0000000000017, 0, 0),
                //0.00000000000000166
                // RamseteCommand passes volts to the callback
                driveTrain::tankDriveVolts,
                driveTrain);
            driveTrain.resetOdometry(trajectory.getInitialPose());


            //     ramseteCommand = new RamseteCommand(
            //         trajectory,
            //         driveTrain::getPose,
            //         new RamseteController(AutonomousConstants.kRamseteB, AutonomousConstants.kRamseteZeta),
            //         new SimpleMotorFeedforward(DriveConstants.ksVolts,
            //                                    DriveConstants.kvVoltSecondsPerMeter,
            //                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            //         DriveConstants.kDriveKinematics,
            //         driveTrain::getWheelSpeeds, //adjusted getWheelSpeeds first
            //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
            //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
            //         // RamseteCommand passes volts to the callback
            //         driveTrain::tankDriveVolts,
            //         driveTrain);
            // driveTrain.resetOdometry(trajectory.getInitialPose());


            // chooser.setDefaultOption("Default Auto", complexAuto);
            // chooser.addOption("Simple Auto", simpleAuto);

            // Shuffleboard.getTab("Autonomous").add(chooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      // DriveTrain Buttons
      new JoystickButton(curveStick, OIConstants.driveHighGearButtonID).whenPressed(new HighGear(driveTrain));
      new JoystickButton(throttleStick, OIConstants.driveLowGearButtonID).whenPressed(new LowGear(driveTrain));

      // Climb Buttons
      // Manual
      new JoystickButton(buttonStick, OIConstants.climbElevatorManualButtonID);
      // .toggleWhenPressed(new ManualElevator(climb));
      new JoystickButton(buttonStick, OIConstants.climbWinchManualButtonID).toggleWhenPressed(new ManualWinch(climb));
      new JoystickButton(buttonStick, OIConstants.climbManualButtonID).toggleWhenPressed(new ManualClimb(climb));
      // Automated
      //new JoystickButton(buttonStick, OIConstants.climbPresetButtonID).whenPressed(new PresetClimb(climb));
      //new JoystickButton(buttonStick, OIConstants.climbUpButtonID).whenPressed(new LevelClimb(climb));
      // new JoystickButton(buttonStick, OIConstants.climbEndButtonID)
      // .whenPressed(new LiftOff(climb));

      // Scoop Arm Buttons
      // Manual
      new JoystickButton(buttonStick, OIConstants.scoopManualButtonID).toggleWhenPressed(new ScoopManual(scoop));
      // Automated
      new JoystickButton(buttonStick, OIConstants.scoopIntakePosButtonID).whileHeld(new ScoopIntakePos(scoop));
      new JoystickButton(buttonStick, OIConstants.scoopLoadingStationButtonID)
              .whileHeld(new ScoopLoadingStationPos(scoop));
      new JoystickButton(buttonStick, OIConstants.scoopLowGoalButtonID).whileHeld(new ScoopLowGoalPos(scoop));

      // Scoop Intake/Expel Buttons
      new JoystickButton(buttonStick, OIConstants.scoopCollectButtonID).whileHeld(new ScoopIntake(scoop));
      new JoystickButton(buttonStick, OIConstants.scoopExpelButtonID).whileHeld(new ScoopExpel(scoop));

      // Control Panel Buttons
      new JoystickButton(buttonStick, OIConstants.controlPanelActivateButtonID)
              .toggleWhenPressed(new Activate(controlPanel));
      new JoystickButton(buttonStick, OIConstants.controlPanelRotationButtonID).whenPressed(new Rotation(controlPanel));
    //   new JoystickButton(buttonStick, OIConstants.controlPanelPositionButtonID)
    //           .toggleWhenPressed(new Position(controlPanel));
  }

  public static double getClimbY() {
      return buttonStick.getY();
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return chooser.getSelected();
            return galacticSearchPathA;
            //return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
            //return ramseteCommand.andThen(() -> scoop.setScoopArmPos(ScoopConstants.scoopArmLowGoalPosition));
            //return ramseteCommand.andThen(new ScoopLowGoalPos(scoop));//, new ScoopExpel(scoop).withTimeout(1));
            //return ramseteCommand.andThen(new ScoopLowGoalPos(scoop));
            

  }


}
