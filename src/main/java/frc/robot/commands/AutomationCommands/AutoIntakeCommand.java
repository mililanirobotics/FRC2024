package frc.robot.commands.AutomationCommands;

//subsystems and commands
import frc.robot.subsystems.IntakeRollerSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeCommand extends Command {
    //declaring subsystems
    private IntakeRollerSubsystem m_rollerSubsystem;
    private DigitalInput m_intakeSwitch;

    private boolean didContact;
    private double intakePower;

    //constructor
    public AutoIntakeCommand(IntakeRollerSubsystem rollerIntakeSubsystem, DigitalInput limitSwitch, double intakePower) {
        //sets the condition to false everytime the command is ran
        didContact = false;
        this.intakePower = intakePower;

        //initializing hardware
        m_intakeSwitch = limitSwitch;

        //initializing subsystems
        m_rollerSubsystem = rollerIntakeSubsystem;
        addRequirements(m_rollerSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("Intake command started");
        System.out.println("Switch Pressed: "+m_intakeSwitch.get());

        //runs both the conveyor and the intake to help guide the piece
        m_rollerSubsystem.setIntakeSpeed(intakePower);

        //sets the condition to false everytime the command is ran
        if(didContact) {
            didContact = false;
        }
    }

    @Override
    public void execute() {
        //updates the condition to true once the note has contacted the switch
        if(m_intakeSwitch.get()) {
            didContact = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake command finished");
        m_rollerSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        //finishes the command once the switch is released
        return didContact && !m_intakeSwitch.get();
    }
}