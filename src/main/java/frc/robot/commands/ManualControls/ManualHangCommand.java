package frc.robot.commands.ManualControls;

//subsystems
import frc.robot.subsystems.HangSubsystem;
//commands 
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.JoystickConstants;
//general imports
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Default command used to manually control the hang.
 * Overrides the automated hang commands if they're scheduled 
 */
public class ManualHangCommand extends Command {
    //declaring subsystems
    private HangSubsystem m_hangSubsystem;
    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualHangCommand(HangSubsystem hangSubsystem, GenericHID joystick) {
        //initializing hardware
        this.joystick = joystick;
        //initializing subsystems
        m_hangSubsystem = hangSubsystem;
        addRequirements(m_hangSubsystem);
    }
    
    @Override
    public void execute() { 
        //sets the conveyor speed based on the secondary gamepad's right joystick's y-value
        double speed = -joystick.getRawAxis(JoystickConstants.kRightYJoystickPort);

        //sets the speed to 0 if the input doesn't pass the minimum value
        if(Math.abs(speed) <= JoystickConstants.kDeadzone) {
            speed = 0;
        }
        
        //setting speeds
        m_hangSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down motors
        m_hangSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}