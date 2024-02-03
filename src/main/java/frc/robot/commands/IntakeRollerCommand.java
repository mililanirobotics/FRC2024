package frc.robot.commands;

//subsystems and commands
import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//general imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerIntakeConstants;

public class IntakeRollerCommand extends Command {
    //declaring subsystems
    private IntakeRollerSubsystem m_IntakeConveyorSubsystem;

    //speed
    // private double greenSpeed;
    // private double blueSpeed;

    //constructor
    public IntakeRollerCommand(IntakeRollerSubsystem intakeConveyorSubsystem) {

        m_IntakeConveyorSubsystem = intakeConveyorSubsystem;
        // blueSpeed = 0;
        // greenSpeed = 0;

        addRequirements(m_IntakeConveyorSubsystem);
    }
    
    @Override
    public void execute() { 
        // if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)) {
        //     greenSpeed += 0.1;
        // }
        // else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)) {
        //     greenSpeed -= 0.1;
        // }
        // else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
        //     blueSpeed += 0.1;
        // }
        // else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
        //     blueSpeed -= 0.1;
        // }
        // else if(joystick.getRawButtonPressed(JoystickConstants.kRightBumperPort)) {
        //     blueSpeed = 0;
        //     greenSpeed = 0;
        // }

        // System.out.println(blueSpeed);
        // SmartDashboard.putNumber("Green speed", greenSpeed);
        // SmartDashboard.putNumber("Blue speed", blueSpeed);
        // SmartDashboard.updateValues();

        m_IntakeConveyorSubsystem.setIntakeSpeed(RollerIntakeConstants.kIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeConveyorSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return m_IntakeConveyorSubsystem.isRollerBeamBroken();
    }
}