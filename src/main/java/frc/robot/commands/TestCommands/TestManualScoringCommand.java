package frc.robot.commands.TestCommands;

//subsystems and commands
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//general imports
import frc.robot.Constants.JoystickConstants;

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
         *  Start buttom = both speeds set to 0
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

        //prints the speed of the launch motors
        SmartDashboard.putNumber("Top launch roller speed", topLaunchRollerSpeed);
        SmartDashboard.putNumber("Bottom launch roller speed", bottomLaunchRollerSpeed);
        SmartDashboard.updateValues();

        if(joystick.getRawButtonPressed(JoystickConstants.kStartButtonPort)) {
            m_scoringSubsystem.setSpeed(bottomLaunchRollerSpeed, topLaunchRollerSpeed);
        }
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