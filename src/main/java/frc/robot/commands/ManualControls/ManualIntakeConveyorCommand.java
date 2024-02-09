package frc.robot.commands.ManualControls;

import frc.robot.subsystems.IntakeConveyorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
//general imports
import frc.robot.Constants.JoystickConstants;

public class ManualIntakeConveyorCommand extends Command {
    //declaring subsystems
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
    //declaring the joystick used
    private GenericHID joystick;

    //constructor
    public ManualIntakeConveyorCommand(GenericHID joystick, IntakeConveyorSubsystem intakeConveyorSubsystem) {
        this.joystick = joystick;
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;

        addRequirements(m_intakeConveyorSubsystem);
    }
    
    @Override
    public void execute() { 
        //sets the conveyor speed based on the secondary gamepad's left joystick value
        double speed = joystick.getRawAxis(JoystickConstants.kLeftYJoystickPort);

        //setting speeds
        m_intakeConveyorSubsystem.setSpeeds(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeConveyorSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return Math.abs(joystick.getRawAxis(JoystickConstants.kLeftYJoystickPort)) <= JoystickConstants.kDeadzone;
    }
}