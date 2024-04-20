package frc.robot.commands.TestCommands;

//subsystems
import frc.robot.subsystems.ScoringSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.JoystickConstants;
//general imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Default command used to test the individual scoring motors 
 */
public class TestManualScoringCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    //declaring the joystick used
    private GenericHID joystick;
    //physical constants
    private double topLaunchRollerSpeed;
    private double bottomLaunchRollerSpeed;

    //constructor
    public TestManualScoringCommand(GenericHID joystick, ScoringSubsystem scoringSubsystem) {
        //initializing hardware and variables
        this.joystick = joystick;
        topLaunchRollerSpeed = 0;
        bottomLaunchRollerSpeed = 0;
        //initializing subsystems
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void execute() { 
        /**
         * Controls: 
         *  A = both speeds +0.1
         *  B = both speeds +0.05
         *  X = both speeds -0.1
         *  Y = both speeds -0.05
         * 
         *  A + right bumper = bottom roller speed +0.1
         *  B + right bumper = bottom roller speed +0.05
         *  X + right bumper = bottom roller speed -0.1
         *  Y + right bumper = bottom roller speed -0.05
         * 
         *  A + left bumper = top roller speed +0.1
         *  B + left bumper = top roller speed +0.05
         *  X + left bumper = top roller speed -0.1
         *  Y + left bumper = top roller speed -0.05
         * 
         *  Start button = sets the speeds of the motors
         *  Back button = both speeds set to 0
         */
        if(joystick.getRawButton(JoystickConstants.kRightBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kAButtonPort) 
        ) {
            bottomLaunchRollerSpeed += 0.1;
        }
        else if(joystick.getRawButton(JoystickConstants.kRightBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)
        ) {
            bottomLaunchRollerSpeed += 0.05;
        }
        else if(joystick.getRawButton(JoystickConstants.kRightBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kXButtonPort) 
        ) {
            bottomLaunchRollerSpeed -= 0.1;
        }
        else if(joystick.getRawButton(JoystickConstants.kRightBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kYButtonPort) 
        ) {
            bottomLaunchRollerSpeed -= 0.05;
        }
        else if(joystick.getRawButton(JoystickConstants.kLeftBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)     
        ) {
            topLaunchRollerSpeed += 0.1;
        }
        else if(joystick.getRawButton(JoystickConstants.kLeftBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)
        ) {
            topLaunchRollerSpeed += 0.05;
        }
        else if(joystick.getRawButton(JoystickConstants.kLeftBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)     
        ) {
            topLaunchRollerSpeed -= 0.1;
        }
        else if(joystick.getRawButton(JoystickConstants.kLeftBumperPort)
            && joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)
        ) {
            topLaunchRollerSpeed -= 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)) {
            bottomLaunchRollerSpeed += 0.1;
            topLaunchRollerSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)) {
            bottomLaunchRollerSpeed += 0.05;
            topLaunchRollerSpeed += 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
            bottomLaunchRollerSpeed -= 0.1;
            topLaunchRollerSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
            bottomLaunchRollerSpeed -= 0.05;
            topLaunchRollerSpeed -= 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBackButtonPort)) {
            bottomLaunchRollerSpeed = 0;
            topLaunchRollerSpeed = 0;
        }

        //sets the speeds 
        if(joystick.getRawButtonPressed(JoystickConstants.kStartButtonPort)) {
            m_scoringSubsystem.setSpeed(bottomLaunchRollerSpeed, topLaunchRollerSpeed);
        }

        //puts speeds on SmartDashboard
        SmartDashboard.putNumber("Top launch roller speed", topLaunchRollerSpeed);
        SmartDashboard.putNumber("Bottom launch roller speed", bottomLaunchRollerSpeed);
        SmartDashboard.updateValues();
    }

    @Override
    public void end(boolean interrupted) {
        m_scoringSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return false;
    }
}