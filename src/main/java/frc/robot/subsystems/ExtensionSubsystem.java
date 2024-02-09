package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//general imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//constants
import frc.robot.Constants.ExtensionConstants;

public class ExtensionSubsystem extends SubsystemBase {
    //solenoids
    private DoubleSolenoid leftExtension;
    private DoubleSolenoid rightExtension;
    //checks if the solenoid is in the forward state
    private boolean isExtended;

    //constructor
    public ExtensionSubsystem() {
        //initializing solenoids
        leftExtension = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            ExtensionConstants.kLeftExtensionForwardChannel, 
            ExtensionConstants.kLeftExtensionReverseChannel
        );

        rightExtension = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            ExtensionConstants.kRightExtensionForwardChannel, 
            ExtensionConstants.kRightExtensionReverseChannel
        );

        //setting the default state to reverse
        leftExtension.set(Value.kReverse);
        rightExtension.set(Value.kReverse);
        isExtended = false;
    }

    /**
     * Retracts the extensions
     */
    public void retract() {
        leftExtension.set(Value.kReverse);
        rightExtension.set(Value.kReverse);
        isExtended = false;
    }

    /**
     * Extends the extensions
     */
    public void extend() {
        leftExtension.set(Value.kForward);
        rightExtension.set(Value.kForward);
        isExtended = true;
    }

    public boolean isExtended() {
        return isExtended;
    }
  
    @Override
    public void periodic() {
        //prints the state of the pistons on Smartdashboard
        SmartDashboard.putBoolean("Is Extended", isExtended);
    }
}