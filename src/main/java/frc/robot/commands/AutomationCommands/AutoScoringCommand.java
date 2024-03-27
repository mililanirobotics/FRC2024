package frc.robot.commands.AutomationCommands;

import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoScoringCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    //declaring variables
    private boolean passedFront;
    private boolean passedBack;

    //constructor
    public AutoScoringCommand(ScoringSubsystem scoringSubsystem) {
        //sets the condition to false everytime the command is ran
        //initializing subsystems
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        passedFront = false;
        passedBack = false;
        System.out.println("Scoring command started");
        m_scoringSubsystem.setSpeed(ScoringConstants.kBotRollerSpeed, ScoringConstants.kTopRollerSpeed);
    }

    @Override
    public void execute() {
        //updates the boolean once the IR sensor is triggered
        if(!m_scoringSubsystem.getStopSensorReading()) {
            passedFront = true;
        }
        if (passedFront && m_scoringSubsystem.getStopSensorReading()) {
            passedBack = true;
        }

        System.out.println("Sensor Reading: "+m_scoringSubsystem.getStopSensorReading());
        System.out.println("Passed Front: "+passedFront);
        System.out.println("Passed Back: "+passedBack);
    }

    @Override
    public void end(boolean interrupted) {
        
        m_scoringSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        //stops the command once the note has fully passed the IR sensor
        return passedBack && m_scoringSubsystem.getStopSensorReading();
    }
}