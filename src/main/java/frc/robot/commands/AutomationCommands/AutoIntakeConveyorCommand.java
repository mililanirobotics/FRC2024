package frc.robot.commands.AutomationCommands;

import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeConveyorCommand extends Command {
    //declaring subsystems
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    private ScoringSubsystem m_scoringSubsystem;
    //declaring variables
    private boolean passedFront;
    private boolean passedBack;
    private boolean seenMiddle;

    //constructor
    public AutoIntakeConveyorCommand(IntakeConveyorSubsystem intakeConveyorSubsystem, ScoringSubsystem scoringSubsystem) {
        //sets the condition to false everytime the command is ran\
        //initializing subsystems
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_intakeConveyorSubsystem, m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        passedFront = false;
        passedBack = false;
        seenMiddle = false;
        System.out.println("Scoring command started");
        m_intakeConveyorSubsystem.setSpeeds(1);
        m_scoringSubsystem.setSpeed(0.3, 0.3);
    }

    @Override
    public void execute() {
        //updates the boolean once the IR sensor is triggered
        if(!m_intakeConveyorSubsystem.getStopSensorReading()) {
            passedFront = true;
            // m_intakeConveyorSubsystem.setInRobot(true);
        }
        if (passedFront && m_intakeConveyorSubsystem.getStopSensorReading()) {
            passedBack = true;
        }

        System.out.println("Sensor Reading: "+m_intakeConveyorSubsystem.getStopSensorReading());
        System.out.println("Passed Front: "+passedFront);
        System.out.println("Passed Middle: "+seenMiddle);
        System.out.println("Passed Back: "+passedBack);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeConveyorSubsystem.shutdown();
        m_scoringSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        //stops the command once the note has fully passed the IR sensor
        return passedBack && m_intakeConveyorSubsystem.getStopSensorReading();
    }
}