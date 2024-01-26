// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class DriverConstants {

  }

  public static class IntakeConveyorConstants {
    public static final int kBottomConveyorPort = 3;
    public static final int kTopConveyorPort = 2;
    public static final int kIntakeIRBeamPort = 0;

    public static final boolean kBottomConveyorReverse = false;
    public static final boolean kTopConveyorReverse = false;

    public static final double kIntakeSpeed = 0.1; // Static speed for the Intake (Currently in testing)
    public static final double kGroundHeight = 1.75; // Height of the rollers from the ground
  }

  public static class VerticalConveyorConstants {
    public static final int kFrontConveyorPort = 4;
    public static final int kBackConveyorPort = 5;
    public static final int kVerticalIRBeamPort = 1;

    public static final boolean kFrontConveyorReversed = false;
    public static final boolean kBackConveyorReversed = true;

    public static final double kConveyorSpeed = 0.1; // Static value for the Vertical Conveyor Speed (Currently in testing)
    
  }

  public final class LimeLightConstants {
    public static final double kMountHeight = 0; //Placeholder
    public static final double kMountAngle = 0; //Placeholder

    public static final double kTargetAMPHeight = 52.25; // Height of the center of the AMP's AprilTag from the ground 
    
  }

  public final class JoystickConstants {
    //joystick port for the gamepad
    public static final int kPrimaryPort = 0;

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
}
