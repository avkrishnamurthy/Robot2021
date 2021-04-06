/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveConstants {
        public static final int driveLeftFrontTalonID = 2;
        public static final int driveLeftBackTalonID = 0;
        public static final int driveRightFrontTalonID = 3;
        public static final int driveRightBackTalonID = 1;

        public static final int driveHighGearSolenoidID = 0; 
        public static final int driveLowGearSolenoidID = 1; 

        public static final int[] kLeftEncoderPorts = new int[]{2, 0};
        public static final int[] kRightEncoderPorts = new int[]{3, 1};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;


        public static final double kTrackWidthMeters = 9.090140961492919;//0.54;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);
    
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.152; //6 inches, 0.152 meters
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        public static final boolean kGyroReversed = true;
        public static final double ksVolts = 0.113;
        public static final double kvVoltSecondsPerMeter = .0565;    
        public static final double kaVoltSecondsSquaredPerMeter = .00371;
        public static final double kPDriveVel = .159;

        public static final double drivePValue = 1.3;
        public static final double driveIValue = 0;
        public static final double driveDValue = 0;
        public static final int driveHighGearVelocity = 0;
        public static final int driveHighGearAcceleration = 0;

        public static final int driveEncoderMOE = 100;

    }

    public static class ClimbConstants {
        public static final int climbElevatorTalonID = 4;
        public static final int climbWinchTalonID = 5;

        public static final int climbWinchLockSolenoidID = 2;

        public static final double winchMultiplierConstant = .98;
        public static final double winchRadius = 1.5;
        public static final double winchStrapWidth = 1/16.0;
        public static final double elevatorRadius = .25;
        public static final double winchRPM = 6380;
        public static final double elevatorRPM = 9000;
        public static final double winchGear = 1/50.0;
        public static final double elevatorGear = 1/25.0; 

  
        public static final int climbElevatorBottomPosition = 0; 
        public static final int climbWinchContractedPosition = 6000; 
        public static final int climbElevatorClimbPosition = 30490;
        public static final int climbWinchClimbPosition = 329993; 
        public static final int climbElevatorHookHeightPosition = 11513;
        public static final int climbElevatorReleaseHookPosition = 26000;
        public static final int climbWinchFinishClimbPosition = 4782;
        public static final int climbWinchMaxHeight = 452513;
        public static final int climbElevatorMaxHeight = 36412;

        // Elevator 
        public static final int climbElevatorMOE = 100;
        public static final double climbElevatorPValue = 1.3;
        public static final double climbElevatorIValue = 0;
        public static final double climbElevatorDValue = 0;
        public static final int climbElevatorVelocity = 2000;
        public static final int climbElevatorAcceleration = 1500;
        // Winch
        public static final int climbWinchMOE = 100;
        public static final double climbWinchPValue = 0.5;
        public static final double climbWinchIValue = 0;
        public static final double climbWinchDValue = 0;
        public static final int climbWinchVelocity = 26000;
        public static final int climbWinchAcceleration = 20500;
    }

    public static class ScoopConstants {
        public static final int scoopArmTalonID = 6;
        public static final int topIntakeTalonID = 7;
        public static final int bottomIntakeTalonID = 8;

        //Assuming 0 position is all the way back (starting a match position)
        public static final int scoopArmIntakePosition = -49250;
        public static final int scoopArmLoadingStationPosition = 0;
        //public static final int scoopArmLevelPosition = -8200; //-9063
        public static final int scoopArmLowGoalPosition = -8200; //-14626
  
        public static final int scoopArmMOE = 100;
        public static final double scoopArmPValue = 1.3;
        public static final double scoopArmIValue = 0;
        public static final double scoopArmDValue = 0;
        public static final int scoopArmVelocity = 3000; //3000
        public static final int scoopArmAcceleration = 2250; //2250

    }

    public static final class OIConstants {
        public static final int throttleJoystickID = 0;
        public static final int curveJoystickID = 1;
        public static final int buttonJoystickID = 2;

         // ----- Buttons -----
        // Curve Stick
        public static final int isQuickTurnButtonID = 2;
  
        // Throttle Stick
        public static final int driveHighGearButtonID = 1;
        public static final int driveLowGearButtonID = 1;

        //Button Stick

        //----- Scoop Intake/Expel Button IDs -----
        public static final int scoopExpelButtonID = 3;
        public static final int scoopCollectButtonID = 6;
  
        //----- Scoop Position Button IDs -----
        public static final int scoopManualButtonID = 1; //Manual
        public static final int scoopLowGoalButtonID = 2;
        public static final int scoopIntakePosButtonID = 4;
        public static final int scoopLoadingStationButtonID = 5;
  
        //----- Control Panel Button IDs -----
        public static final int controlPanelActivateButtonID = 7;
        public static final int controlPanelRotationButtonID = 8;
        public static final int controlPanelPositionButtonID = 9;

        //----- Climb Button IDs -----
        public static final int climbPresetButtonID = 3;
        public static final int climbUpButtonID = 6;
        public static final int climbEndButtonID = 8;
        public static final int climbElevatorManualButtonID = 9; //Manual
        public static final int climbWinchManualButtonID = 10; //Manual
        public static final int climbManualButtonID = 11; //Manual
      }

    public static class ControlPanelConstants {
        public static final int controlPanelTalonID = 9;

        public static final int controlPanelExtendedSolonoidID = 3;
        public static final int controlPanelRetractedSolonoidID = 4;

        public static final double controlPanelPValue = 1.3;
        public static final double controlPanelIValue = 0;
        public static final double controlPanelDValue = 0;
        public static final int controlPanelVelocity = 4000;
        public static final int controlPanelAcceleration = 3500;
        public static final int controlPanelRotationPosition = 212966;
        public static final int controlPanelNextColorPosition = 8191;
        //controlPanelCircumference = 6.28 inches
        //Each color wedge is 12.5 inches
        //Two rotations to advance one color
        //Andymark redline motor
        //4:1 gearbox ratio
        //4096 cpr encoder  
    }
    public static class AutonomousConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.5; //21.6 * 0.3048;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1; //23.9 * 0.3048;

        public static final double kAutoDriveDistanceMeters = 0.305; //12 inches
        public static final double kAutoDriveSpeed = 0.2; 
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
