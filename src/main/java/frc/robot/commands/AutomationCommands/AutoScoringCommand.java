package frc.robot.commands.AutomationCommands;

//subsystems
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.ScoringConstants;


/**
 * Automatically scores the note once aligned with the amp.
 * Tracks the state of the note through the IR sensor mounted in the hood, 
 * allowing for automatic starting/stopping of the payload
 */
public class AutoScoringCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    //declaring variables
    private boolean passedFront;
    private boolean passedBack;

    //constructor
    public AutoScoringCommand(IntakeConveyorSubsystem intakeConveyorSubsystem, ScoringSubsystem scoringSubsystem) {
        //sets the condition to false everytime the command is ran
        //initializing subsystems
        m_scoringSubsystem = scoringSubsystem;
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        //setting all states to false by default
        passedFront = false;
        passedBack = false;
        //printing initialize statement 
        System.out.println("Scoring command started");
        //turning on scorer
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
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down the scorer and updating the state of the note in the robot
        m_intakeConveyorSubsystem.setNoteIn(false);
        m_intakeConveyorSubsystem.setNotePayload(false);
        m_scoringSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        //stops the command once the note has fully passed the IR sensor
        return passedBack && m_scoringSubsystem.getStopSensorReading();
    }
} 