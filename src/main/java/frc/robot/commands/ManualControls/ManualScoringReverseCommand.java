package frc.robot.commands.ManualControls;

//subsystems
import frc.robot.subsystems.ScoringSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ScoringConstants;
//general imports
import edu.wpi.first.wpilibj.GenericHID;


/**
 * Command used to manually activate the scoring motors in the reverse direction 
 * when the X button is pressed on the secondary gamepad 
 */
public class ManualScoringReverseCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualScoringReverseCommand(GenericHID joystick, ScoringSubsystem scoringSubsystem) {
        //initializing hardware
        this.joystick = joystick;
        //initializing subsystems
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        //printing initializing statement
        System.out.println("Manual Scoring Reverse Started");
        //setting the speeds of the motors 
        m_scoringSubsystem.setSpeed(-1, -ScoringConstants.kTopRollerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down the motors and printing an update
        System.out.println("Manual Scoring Reverse Ended");
        m_scoringSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        //ends the command once the X button is released
        return !joystick.getRawButton(JoystickConstants.kXButtonPort);
    }
}