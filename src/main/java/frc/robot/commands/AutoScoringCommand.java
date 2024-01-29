package frc.robot.commands;

import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoScoringCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;

    private boolean didContact;

    //constructor
    public AutoScoringCommand(ScoringSubsystem scoringSubsystem) {
        didContact = false;

        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("Command Started");
        m_scoringSubsystem.setSpeed(ScoringConstants.botRollerSpeed, ScoringConstants.topRollerSpeed);
    }

    @Override
    public void execute() {
        //updates the boolean once the limit switch is triggered
        if(m_scoringSubsystem.getSensorReading()) {
            didContact = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_scoringSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return didContact && !m_scoringSubsystem.getSensorReading();
    }
}