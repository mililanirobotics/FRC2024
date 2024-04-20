package frc.robot.commands.AutomationCommands;

//subsystems
import frc.robot.subsystems.HangSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Automatically retracts the hang into its default position
 */
public class AutoHangRetractCommand extends Command {
    //declaring subsystems
    private HangSubsystem m_hangSubsystem;

    //constructor
    public AutoHangRetractCommand(HangSubsystem hangSubsystem) {
        //initializing subsystems
        m_hangSubsystem = hangSubsystem;
        addRequirements(m_hangSubsystem);
    }
    
    @Override
    public void initialize() {
        //printing initialize statement 
        System.out.println("Lifting hang");
        //setting hang power to max (reverse)
        m_hangSubsystem.setSpeed(-1);
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down hang and setting its state to retracted
        m_hangSubsystem.shutdown();
        m_hangSubsystem.setHangState(false);
    }

    @Override
    public boolean isFinished() {
        //stops the hang once the IR sensor holes are lined up
        return m_hangSubsystem.getHangSensorReading();
    }
}