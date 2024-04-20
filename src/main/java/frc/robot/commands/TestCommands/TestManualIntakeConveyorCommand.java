package frc.robot.commands.TestCommands;

//subsystems
import frc.robot.subsystems.IntakeConveyorSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.JoystickConstants;
//general imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Default command used to test the intake and conveyors 
 */
public class TestManualIntakeConveyorCommand extends Command {
    //declaring subsystems
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    //declaring the joystick used
    private GenericHID joystick;
    //speed of the conveyor
    private double intakeConveyorSpeed;

    //constructor
    public TestManualIntakeConveyorCommand(GenericHID joystick, IntakeConveyorSubsystem intakeConveyorSubsystem) {
        //initializing hardware and variables
        this.joystick = joystick;
        intakeConveyorSpeed = 0;
        //initializing subsystems 
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        addRequirements(m_intakeConveyorSubsystem);
    }
    
    @Override
    public void execute() { 

        /**
         * Controls: 
         *  A = both speeds +0.1
         *  B = both speeds +0.05
         *  X = both speeds -0.1
         *  Y = both speeds -0.05
         * 
         *  Start buttom = sets the speeds of the motor
         *  Back button = both speeds 0
         */
        if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)     
        ) {
            intakeConveyorSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)
        ) {
            intakeConveyorSpeed += 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
            intakeConveyorSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
            intakeConveyorSpeed -= 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBackButtonPort)) {
            intakeConveyorSpeed = 0;
        }

        //sets the speeds
        if(joystick.getRawButtonPressed(JoystickConstants.kStartButtonPort)) {
            m_intakeConveyorSubsystem.setSpeeds(intakeConveyorSpeed);
        }

        //puts speeds on SmartDashboard
        SmartDashboard.putNumber("Conveyor and Intake Speed", intakeConveyorSpeed);
        SmartDashboard.updateValues();
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