package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConveyorConstants;

public class IntakeConveyorSubsystem extends SubsystemBase{
    private CANSparkMax bottomConveyor;
    private CANSparkMax topConveyor;
    private DigitalInput IntakeBreakBeam;

    public IntakeConveyorSubsystem() {
      bottomConveyor = new CANSparkMax(IntakeConveyorConstants.kBottomConveyorPort, MotorType.kBrushless);
      topConveyor = new CANSparkMax(IntakeConveyorConstants.kTopConveyorPort, MotorType.kBrushless);
      IntakeBreakBeam = new DigitalInput(IntakeConveyorConstants.kIntakeIRBeamPort);

      bottomConveyor.setInverted(IntakeConveyorConstants.kBottomConveyorReverse);
      topConveyor.setInverted(IntakeConveyorConstants.kTopConveyorReverse);
    }

    /*
     * Sets speed method for the Intake, runs at a static speed in the command
     */
    public void setIntakeSpeed(double percentPower) {
      bottomConveyor.set(percentPower);
      topConveyor.set(percentPower);
    }

    /*
     * Testing method for adjusting roller speeds
     */
    public void setIndependentSpeed(double blueS, double greenS) {
      bottomConveyor.set(greenS);
      topConveyor.set(blueS);
    }

    public void shutdown() {
      bottomConveyor.set(0);
      topConveyor.set(0);
    }

    /*
     * Returns the breakbeam's condition of being broken or not
     */
    public boolean isIntakeBeamBroken() {
      return IntakeBreakBeam.get();
    }

    @Override
    public void periodic() {

    }
}