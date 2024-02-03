package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase{
    private CANSparkMax frontConveyor;
    private CANSparkMax backConveyor;
    private DigitalInput verticalBreakBeam;

    public ConveyorSubsystem () {
        frontConveyor = new CANSparkMax(ConveyorConstants.kFrontConveyorPort, MotorType.kBrushless);
        backConveyor = new CANSparkMax(ConveyorConstants.kBackConveyorPort, MotorType.kBrushless);

        verticalBreakBeam = new DigitalInput(ConveyorConstants.kVerticalIRBeamPort);
    }

    public void setConveyorSpeed(double percentPower) {
        frontConveyor.set(percentPower);
        backConveyor.set(percentPower);
    }

    public void shutdown() {
        frontConveyor.set(0);
        backConveyor.set(0);
    }

    public boolean isVerticalBeamBroken() {
        return verticalBreakBeam.get();
    }

}