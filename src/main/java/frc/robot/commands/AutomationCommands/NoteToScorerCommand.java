package frc.robot.commands.AutomationCommands;

import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

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
        m_scoringSubsystem.setSpeed(ScoringConstants.kMovingSpeed, ScoringConstants.kMovingSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_scoringSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        //stops the command once the note has fully passed the IR sensor
        return !m_scoringSubsystem.getStopSensorReading();
    }
}