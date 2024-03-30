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

    //constructor
    public ExtensionSubsystem() {
        //initializing solenoids
        leftExtension = new DoubleSolenoid(
            1,
            PneumaticsModuleType.REVPH, 
            ExtensionConstants.kLeftExtensionForwardChannel, 
            ExtensionConstants.kLeftExtensionReverseChannel
        );

        rightExtension = new DoubleSolenoid(
            1,
            PneumaticsModuleType.REVPH, 
            ExtensionConstants.kRightExtensionForwardChannel, 
            ExtensionConstants.kRightExtensionReverseChannel
        );

        //setting the default state to reverse
        leftExtension.set(Value.kForward);
        rightExtension.set(Value.kForward);
    }

    public DoubleSolenoid.Value getLeftState() {
        return leftExtension.get();
    }

    public DoubleSolenoid.Value getRightState() {
        return rightExtension.get();
    }

    /**
     * Retracts the extensions
     */
    public void retract() {
        leftExtension.set(Value.kForward);
        rightExtension.set(Value.kForward);
    }

    /**
     * Extends the extensions
     */
    public void extend() {
        leftExtension.set(Value.kReverse);
        rightExtension.set(Value.kReverse);
    }

    public boolean isExtended() {
        return leftExtension.get() == Value.kReverse && rightExtension.get() == Value.kReverse;
    }
  
    @Override
    public void periodic() {
        //prints the state of the pistons on Smartdashboard
        SmartDashboard.putBoolean("is left extended", leftExtension.get() == Value.kReverse);
        SmartDashboard.putBoolean("is right extended", rightExtension.get() == Value.kReverse);

    }
    
}