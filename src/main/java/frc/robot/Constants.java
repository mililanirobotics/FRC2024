// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//path planner
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//swerve drive
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//misc
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Swerve constants
  public static class SwerveModuleConstants {
    //physical properties
    public static final double kRotationGearRatio = 1 / (165 / 4.0);
    public static final double kDriveGearRatio = 1 / 5.14;
    public static final double kWheelDiameter = Units.inchesToMeters(3);
    public static final double kRotationToMeters = kDriveGearRatio * Math.PI * kWheelDiameter;
    public static final double kRotationToRadians = kRotationGearRatio * 2 * Math.PI;
    public static final double kMetersPerSecond = kRotationToMeters / 60.0 ;
    public static final double kRadiansPerSecond = kRotationToRadians / 60.0;

    public static final int kRightDriveCurrentLimit = 25;
    public static final int kLeftDriveCurrentLimit = 25;
    public static final int kRotationCurrentLimit = 25;

    // Drive Port Constants
    public static final int kLeftFrontWheelPort = 15;
    public static final int kLeftFrontRotationPort = 14;

    public static final int kRightFrontWheelPort = 9;
    public static final int kRightFrontRotationPort = 8;

    public static final int kLeftBackWheelPort = 19;
    public static final int kLeftBackRotationPort = 18;

    public static final int kRightBackWheelPort = 21;
    public static final int kRightBackRotationPort = 20;

    public static final int kLeftFrontCANCoderPort = 8;
    public static final int kRightFrontCANCoderPort = 11;
    public static final int kLeftBackCANCoderPort = 17;
    public static final int kRightBackCANCoderPort = 2;
    public static final double kLeftFrontCANCoderOffset = -0.181396484375;
    public static final double kRightFrontCANCoderOffset = -0.7412109375;
    public static final double kLeftBackCANCoderOffset = -0.555419921875;
    public static final double kRightBackCANCoderOffset = -0.326904296875;

    public static final int kLeftBackIndex = 2;
    public static final int kRightBackIndex = 3;
    public static final int kLeftFrontIndex = 0;
    public static final int kRightFrontIndex = 1;
    
    // Reverse Booleans
    public static final boolean kLeftFrontDriveReversed = false;
    public static final boolean kRightFrontDriveReversed = true;
    public static final boolean kLeftBackDriveReversed = false;
    public static final boolean kRightBackDriveReversed = true;

    public static final boolean kLeftFrontRotationReversed = false;
    public static final boolean kRightFrontRotationReversed = false;
    public static final boolean kLeftBackRotationReversed = false;
    public static final boolean kRightBackRotationReversed = false;

    public static final boolean kLeftFrontCANCoderReversed = false;
    public static final boolean kRightFrontCANCoderReversed = false;
    public static final boolean kLeftBackCANCoderReversed = false;
    public static final boolean kRightBackCANCoderReversed = false;

    // PID Constants
    public static final double kTurningP = 0.4;
    public static final double kTurningI = 0.001;
    public static final double kTurningD = 0;
    public static final double kTurningTolerance = 0.1;

    // The distances between each Module from the center of the robot (Meters)
    public static final double kModuleDistance = 0.312547;

    // 2d translation coordinates relative to center 
    public static final double kLeftFront2dX = kModuleDistance;
    public static final double kLeftFront2dY = kModuleDistance;

    public static final double kRightFront2dX = kModuleDistance;
    public static final double kRightFront2dY = -kModuleDistance;

    public static final double kLeftBack2dX = -kModuleDistance;
    public static final double kLeftBack2dY = kModuleDistance;

    public static final double kRightBack2dX = -kModuleDistance;
    public static final double kRightBack2dY = -kModuleDistance;

    public static final double kTrackWidth = kModuleDistance * 2;

    public static final Translation2d translationLength = new Translation2d(
      kTrackWidth,
      kTrackWidth
    );

    public static final Translation2d leftFrontLocation = new Translation2d(
      SwerveModuleConstants.kLeftFront2dX, 
      SwerveModuleConstants.kLeftFront2dY
    );

    public static final Translation2d rightFrontLocation = new Translation2d(
      SwerveModuleConstants.kRightFront2dX,
      SwerveModuleConstants.kRightFront2dY
    );

    public static final Translation2d leftBackLocation = new Translation2d(
      SwerveModuleConstants.kLeftBack2dX,
      SwerveModuleConstants.kLeftBack2dY
    );

    public static final Translation2d rightBackLocation = new Translation2d(
      SwerveModuleConstants.kRightBack2dX,
      SwerveModuleConstants.kRightBack2dY
    );

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      leftFrontLocation, 
      rightFrontLocation,
      leftBackLocation,
      rightBackLocation
    );

    //drivetrain simulation
    public static final double ksVolts = 0.22;
    public static final double kvVoltsSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;
    
    public static final DCMotor kDriveGearbox = DCMotor.getNEO(1);

    public static final LinearSystem<N2, N2, N2> kDrivePlant = 
      LinearSystemId.identifyDrivetrainSystem(
        kvVoltsSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian,
        kaVoltSecondsSquaredPerRadian
      );
  }

  public static class LEDConstants {
    public static final int CANdleID = 30;
    public static final int LEDcount = 69;
    //69
  }

  //constants for the conveyor payload
  public static class IntakeConveyorConstants {
    //ports and directionality of conveyors
    public static final int kLeftConveyorTopRollerPort = 7;
    public static final int kRightConveyorBottomRollerPort = 12;
    public static final boolean kLeftConveyorTopRollerReverse = true;
    public static final boolean kRightConveyorBottomRollerReverse = false; 

    //minimum joystick value to active conveyors
    public static final double kConveyorDeadband = 0.05;

    //IR sensor
    public static final int kStartIntakeConveyorSensorPort = 4;
    public static final int kStopIntakeConveyorSensorPort = 3;
  }

  //constants for the scoring payload
  public static class ScoringConstants {
    //ports and directionality of scoring motors
    public static final int kUpperFlywheelPort = 13;
    public static final int kLowerFlywheelPort = 10;
    public static final boolean kUpperFlywheelReverse = true;
    public static final boolean kLowerFlywheelReverse = true; 
    public static final int kScoringSensorPort = 5;

    public static final double kTopRollerSpeed = 1;
    public static final double kBotRollerSpeed = 0.3;
  }

  //constants for the limelight
  public static class LimeLightConstants {
    public static final double kMountHeight = 0; //Placeholder
    public static final double kMountAngle = 0; //Placeholder

    public static final double kTargetAMPHeight = 52.25; // Height of the center of the AMP's AprilTag from the ground 
  }

  //constants for the extension
  public static class ExtensionConstants {
    public static final int kLeftExtensionForwardChannel = 7;
    public static final int kLeftExtensionReverseChannel = 6;
    public static final int kRightExtensionForwardChannel = 5;
    public static final int kRightExtensionReverseChannel = 4;
  } 

  //joystick constants and buttons
  public static class JoystickConstants {
    public static final int kPrimaryGamepadPort = 0;
    public static final int kSecondaryGamepadPort = 1;
    public static final int kTestingGamepadPort = 2;

    //Gamepad Axis Ports
    public static final int kleftXJoystickPort = 0;
    public static final int kLeftYJoystickPort = 1;
    public static final int kRightXJoystickPort = 4;
    public static final int kRightYJoystickPort = 5; 
    public final static int kLeftTriggerPort = 2;
    public final static int kRightTriggerPort = 3;

    //Gamepad Button Ports
    public final static int kAButtonPort = 1;
    public final static int kBButtonPort = 2;
    public final static int kXButtonPort = 3;
    public final static int kYButtonPort = 4;
    public final static int kLeftBumperPort = 5;
    public final static int kRightBumperPort = 6;
    public final static int kBackButtonPort = 7;
    public final static int kStartButtonPort = 8;   

    //joystick port for the gamepad
    public final static int kPrimaryLeftStickPort = 0;
    public final static int kPrimaryRightStickPort = 1;

    //Dpad values
    public final static int kDpadUp = 0;
    public final static int kDpadDown = 180;

    public final static double kDeadzone = 0.2;

  }

  public static class DriveConstants {
    // Drive Speed Constants
    public static final double kDriveMaxMetersPerSecond = 4.95;
    public static final double kRotationMaxRadiansPerSecond = 0.5;
    public static final double kDriveMetersPerSecondLimit = 1;

    public static final double kTeleDriveMaxAcceleration = kDriveMaxMetersPerSecond * 8;
    public static final double kTeleRotationMaxAngularAcceleration = kRotationMaxRadiansPerSecond * 8;

    //======================
    // auto constants
    //======================

    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDrive = 0;

    //PID Constants for auto
    public static final double kXP = 0;
    public static final double kXI = 0;
    public static final double kXD = 0;
    
    public static final double kYP = 0;
    public static final double kYI = 0;
    public static final double kYD = 0;

    public static final double kThetaP = 0;
    public static final double kThetaI = 0;
    public static final double kThetaD = 0;

    //PID controllers
    public static final PIDController XController = new PIDController(
        DriveConstants.kXP, 
        DriveConstants.kXI, 
        DriveConstants.kXD
    );

    public static final PIDController yController = new PIDController(
        DriveConstants.kYP, 
        DriveConstants.kYI, 
        DriveConstants.kYD
    );

    public static final ProfiledPIDController thetaController = new ProfiledPIDController(
        SwerveModuleConstants.kTurningP,
        SwerveModuleConstants.kTurningI,
        SwerveModuleConstants.kTurningD,
        new TrapezoidProfile.Constraints(
            DriveConstants.kDriveMaxMetersPerSecond, 
            DriveConstants.kTeleDriveMaxAcceleration
        ) 
    );
  }

  //constants for auto paths
  public static class AutoConstants {
    //auto constraints 
    public static final double kAutoDriveMaxMetersPerSecond = 0.05;
    public static final double kAutoDriveMaxAcceleration = kAutoDriveMaxMetersPerSecond * 8;
    public static final double kAutoDriveMaxRadiansPerSecond = 0.01;
    public static final double kAutoDriveMaxAngularAcceleration = kAutoDriveMaxRadiansPerSecond * 8;

    //PID Constants
    public static final double kPController = 8;
    public static final double kIController = 0;
    public static final double kDController = 0.000;

    public static final double kPThetaController = 7; //1.7
    public static final double kIThetaController = 0; 
    public static final double kDThetaController = 0.2;


    //path planner constraint 
    public static final PathConstraints pathConstraints = new PathConstraints(
      kAutoDriveMaxMetersPerSecond, 
      kAutoDriveMaxAcceleration, 
      kAutoDriveMaxRadiansPerSecond, 
      kAutoDriveMaxAngularAcceleration
    );
    
    //max module speed
    public static final double kMaxModuleSpeed = 2;
 
    //Path planner config
    public static final HolonomicPathFollowerConfig pathFollowingConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(kPController, kIController, kDController),
      new PIDConstants(kPThetaController, kIThetaController, kDThetaController),
      kMaxModuleSpeed, 
      SwerveModuleConstants.translationLength.getNorm(),
      new ReplanningConfig() 
    );
  }
}