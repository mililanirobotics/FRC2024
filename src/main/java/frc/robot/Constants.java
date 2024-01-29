// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static class RollerIntakeConstants {
    //ports and directionality of rollers
    public static final int kLowerRollerPort = 6;
    public static final int kUpperRollerPort = 11;
    public static final boolean kLowerRollerReverse = true;
    public static final boolean kUpperRollerReverse = true;

    //minimum joystick value to active intake
    public static final double kIntakeDeadband = 0.05;
  }

  public static class VerticalConveyorConstants {
    public static final int kFrontConveyorPort = 4;
    public static final int kBackConveyorPort = 5;
    public static final int kVerticalIRBeamPort = 1;

    public static final boolean kFrontConveyorReversed = false;
    public static final boolean kBackConveyorReversed = true;

    public static final double kConveyorSpeed = 0.1; // Static value for the Vertical Conveyor Speed (Currently in testing)
    
  }


  public static class ConveyorConstants {
    //ports and directionality of conveyors
    public static final int kLeftConveyorPort = 9;
    public static final int kRightConveyorPort = 8;
    public static final boolean kLeftConveyorReverse = true;
    public static final boolean kRightConveyorReverse = true; 

    //minimum joystick value to active conveyors
    public static final double kConveyorDeadband = 0.05;
  }

  public static class ScoringConstants {
    //ports and directionality of scoring motors
    public static final int kUpperFlywheelPort = 10;
    public static final int kLowerFlywheelPort = 7;
    public static final boolean kUpperFlywheelReverse = true;
    public static final boolean kLowerFlywheelReverse = true; 
    public static final int kScoringSensorPort = 1;

    public static final double topRollerSpeed = 1;
    public static final double botRollerSpeed = 0.3;
  }

  public final class LimeLightConstants {
    public static final double kMountHeight = 0; //Placeholder
    public static final double kMountAngle = 0; //Placeholder

    public static final double kTargetAMPHeight = 52.25; // Height of the center of the AMP's AprilTag from the ground 
  }

  public final class JoystickConstants {
    //joystick port for the gamepad
    public static final int kPrimaryPort = 0;
    public static final int kSecondaryPort = 1;

    //Gamepad ports
    public final static int kAButtonPort = 1;
    public final static int kBButtonPort = 2;
    public final static int kXButtonPort = 3;
    public final static int kYButtonPort = 4;
    public final static int kLeftBumperPort = 5;
    public final static int kRightBumperPort = 6;
    public final static int kBackButtonPort = 7;
    public final static int kStartButtonPort = 8;

    //Gamepad axis ports
    public final static int kLeftYJoystickPort = 1;
    public final static int kLeftTriggerPort = 2;
    public final static int kRightTriggerPort = 3;
    public final static int kRightYJoystickPort = 5;
  }

   public static class SwerveModuleConstants {

    public static final int kDriverControllerPort = 0;

    public static final double kRotationGearRatio = 1 / (150 / 7.0);
    public static final double kDriveGearRatio = 1/ 8.14;
    public static final double kWheelDiameter = Units.inchesToMeters(4);

    // Drive Port Constants
    public static final int kLeftFrontWheelPort = 5;
    public static final int kLeftFrontRotationPort = 4;

    public static final int kRightFrontWheelPort = 12;
    public static final int kRightFrontRotationPort = 13;

    public static final int kLeftBackWheelPort = 3;
    public static final int kLeftBackRotationPort = 2;

    public static final int kRightBackWheelPort = 15;
    public static final int kRightBackRotationPort = 14;

    public static final int kLeftFrontCANCoderPort = 3;
    public static final int kRightFrontCANCoderPort = 1;
    public static final int kLeftBackCANCoderPort = 2;
    public static final int kRightBackCANCoderPort = 4;

    public static final double kLeftFrontCANCoderOffset = -0.56542;
    public static final double kRightFrontCANCoderOffset = -0.29248;
    public static final double kLeftBackCANCoderOffset = -0.4479;
    public static final double kRightBackCANCoderOffset = -0.0014;
    
    
    // Reverse Booleans
    public static final boolean kLeftFrontDriveReversed = false;
    public static final boolean kRightFrontDriveReversed = true;
    public static final boolean kLeftBackDriveReversed = false;
    public static final boolean kRightBackDriveReversed = true;

    public static final boolean kLeftFrontRotationReversed = true;
    public static final boolean kRightFrontRotationReversed = true;
    public static final boolean kLeftBackRotationReversed = true;
    public static final boolean kRightBackRotationReversed = true;

    public static final boolean kLeftFrontCANCoderReversed = false;
    public static final boolean kRightFrontCANCoderReversed = false;
    public static final boolean kLeftBackCANCoderReversed = false;
    public static final boolean kRightBackCANCoderReversed = false;

    // Conversion Units
    public static final double kRotationToMeters = kDriveGearRatio * Math.PI * kWheelDiameter;
    public static final double kRotationToRadians = kRotationGearRatio * 2 * Math.PI;

    // Measurement Units
    public static final double kMetersPerSecond = kRotationToMeters / 60.0 ;
    public static final double kRadiansPerSecond = kRotationToRadians / 60.0;

    // PID Constants
    public static final double kTurningP = 0.4;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    // The distances between each Module from the center of the robot (Meters)
    public static final double kModuleDistance = 0.312;

    // 2d translation coordinates relative to center 
    public static final double kLeftFront2dX = kModuleDistance;
    public static final double kLeftFront2dY = kModuleDistance;

    public static final double kRightFront2dX = kModuleDistance;
    public static final double kRightFront2dY = -kModuleDistance;

    public static final double kLeftBack2dX = -kModuleDistance;
    public static final double kLeftBack2dY = kModuleDistance;

    public static final double kRightBack2dX = -kModuleDistance;
    public static final double kRightBack2dY = -kModuleDistance;

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
  }

  public static class DriveConstants {

    // Drive Speed Constants
    public static final double kDriveMaxMetersPerSecond = 1;
    public static final double kRotationMaxRadiansPerSecond = 1.5;

    public static final double kTeleDriveMaxAcceleration = kDriveMaxMetersPerSecond * 2;
    public static final double kTeleRotationMaxAngularAcceleration = kRotationMaxRadiansPerSecond;
  }
}