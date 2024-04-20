package frc.robot.commands.LEDS;

//subsystems
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;



/**
 * Automatically scores the note once aligned with the amp.
 * Tracks the state of the note through the IR sensor mounted in the hood, 
 * allowing for automatic starting/stopping of the payload
 */
public class ResetSignalCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    private ExtensionSubsystem m_extensionSubsystem;

    //constructor
    public ResetSignalCommand(IntakeConveyorSubsystem intakeConveyorSubsystem, ScoringSubsystem scoringSubsystem, ExtensionSubsystem extensionSubsystem) {
        //sets the condition to false everytime the command is ran
        //initializing subsystems
        m_scoringSubsystem = scoringSubsystem;
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        m_extensionSubsystem = extensionSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        m_intakeConveyorSubsystem.setNoteIn(false);
        m_intakeConveyorSubsystem.setNotePayload(false);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        //stops the command once the note has fully passed the IR sensor
        return true;
    }
} 