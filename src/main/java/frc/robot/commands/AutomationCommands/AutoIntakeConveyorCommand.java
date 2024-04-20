package frc.robot.commands.AutomationCommands;

//subsystems
import frc.robot.subsystems.IntakeConveyorSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Automatically intakes notes and stores them into the scoring hood.
 * The start and stop positions are tracked and controlled via the IR sensors in the intake
 */
public class AutoIntakeConveyorCommand extends Command {
    //declaring subsystems
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    //declaring variables
    private boolean passedFront;
    private boolean passedBack;

    //constructor
    public AutoIntakeConveyorCommand(IntakeConveyorSubsystem intakeConveyorSubsystem) {
        //sets the condition to false everytime the command is ran
        //initializing subsystems
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        addRequirements(m_intakeConveyorSubsystem);
    }
    
    @Override
    public void initialize() {
        //setting all states to false by default
        passedFront = false;
        passedBack = false;
        //printing initialize statement 
        System.out.println("Scoring command started");
        //turning on intake
        m_intakeConveyorSubsystem.setSpeeds(1);
    }

    @Override
    public void execute() {
        //updates the boolean once the IR sensor is triggered
        if(!m_intakeConveyorSubsystem.getStopSensorReading()) {
            passedFront = true;
            m_intakeConveyorSubsystem.setNoteIn(true);
        }
        if(passedFront && m_intakeConveyorSubsystem.getStopSensorReading()) {
            passedBack = true;
            m_intakeConveyorSubsystem.setNotePayload(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down intake
        m_intakeConveyorSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        //stops the command once the note has fully passed the IR sensor
        return passedBack && m_intakeConveyorSubsystem.getStopSensorReading();
    }
}