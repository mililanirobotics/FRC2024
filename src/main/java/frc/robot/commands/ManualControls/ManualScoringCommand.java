package frc.robot.commands.ManualControls;
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//general imports
import edu.wpi.first.wpilibj.GenericHID;
//constants
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ScoringConstants;

public class ManualScoringCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualScoringCommand(GenericHID joystick, ScoringSubsystem scoringSubsystem) {
        this.joystick = joystick;
        
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("Manual Scoring Started");
        m_scoringSubsystem.setSpeed(ScoringConstants.kBotRollerSpeed, ScoringConstants.kTopRollerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Manual Scoring Ended");
        m_scoringSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return joystick.getRawButtonReleased(JoystickConstants.kBButtonPort);
    }
}