package frc.robot.commands.ManualControls;

//subsystems
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ScoringConstants;
//general imports
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Default command used to manually control the intake and conveyor 
 * Overrides automated intake/conveyor commands if they're scheduled
 */
public class ManualIntakeConveyorCommand extends Command {
    //declaring subsystems
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    private ScoringSubsystem m_scoringSubsystem;
    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualIntakeConveyorCommand(GenericHID joystick, IntakeConveyorSubsystem intakeConveyorSubsystem, ScoringSubsystem scoringSubsystem) {
        //initializing hardware
        this.joystick = joystick;
        //initializing subsystems
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        m_scoringSubsystem = scoringSubsystem;
        addRequirements(m_intakeConveyorSubsystem, m_scoringSubsystem);
    }
    
    @Override
    public void execute() { 
        //sets the conveyor speed based on the secondary gamepad's left joystick value
        double speed = -joystick.getRawAxis(JoystickConstants.kLeftYJoystickPort);

        //sets the speed to 0 if the input doesn't pass the minimum value
        if(Math.abs(speed) <= JoystickConstants.kDeadzone) {
            speed = 0;
        }

        //setting speeds
        if(Math.abs(speed) > 0.05) {
            m_scoringSubsystem.setSpeed(ScoringConstants.kMovingSpeed, ScoringConstants.kMovingSpeed);
        }
        else {
            m_scoringSubsystem.setSpeed(0, 0);
        }
        m_intakeConveyorSubsystem.setSpeeds(speed);
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down motors
        m_intakeConveyorSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}