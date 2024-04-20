package frc.robot.commands.ManualControls;

import frc.robot.subsystems.IntakeConveyorSubsystem;
//subsystems
import frc.robot.subsystems.ScoringSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConveyorConstants;
//constants
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ScoringConstants;
//general imports
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Command used to manually activate the scoring motors 
 * when the B button is pressed on the secondary gamepad 
 */
public class ManualScoringCommand extends Command {
    //declaring subsystems
    private ScoringSubsystem m_scoringSubsystem;
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualScoringCommand(GenericHID joystick, ScoringSubsystem scoringSubsystem, IntakeConveyorSubsystem intakeConveyorSubsystem) {
        //initializing hardware
        this.joystick = joystick;
        //initializing subsystems
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_scoringSubsystem);
    }
    
    @Override
    public void initialize() {
        //printing initializing statement
        System.out.println("Manual Scoring Started");
        //setting the speeds of the scoring motors
        m_scoringSubsystem.setSpeed(1, ScoringConstants.kTopRollerSpeed);
    }

    @Override
    public void execute() {
        // if(!m_scoringSubsystem.getStopSensorReading()) {
        //     m_intakeConveyorSubsystem.setNoteIn(false);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down the motors and printing an update
        System.out.println("Manual Scoring Ended");
        m_scoringSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        //ends the command once the B button is released
        return !joystick.getRawButton(JoystickConstants.kBButtonPort);
    }
}