package frc.robot.commands.AutomationCommands;

//subsystems
import frc.robot.subsystems.ScoringSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.ScoringConstants;

/**
 * Turns on the scoring motors until the note reaches the scoring hood's IR sensor.
 * Designed to help guide the note to the hood when intaking notes
 */
public class NoteToScorerCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    
    //constructor
    public NoteToScorerCommand(ScoringSubsystem scoringSubsystem) {
        //initializing subsystems
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        //setting the speed of the scorer
        m_scoringSubsystem.setSpeed(ScoringConstants.kMovingSpeed, ScoringConstants.kMovingSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down the scoring motors
        m_scoringSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        //stops the command once the note has triggered the IR sensor 
        return !m_scoringSubsystem.getStopSensorReading();
    }
}