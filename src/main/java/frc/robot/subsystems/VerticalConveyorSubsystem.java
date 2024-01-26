package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VerticalConveyorConstants;

public class VerticalConveyorSubsystem extends SubsystemBase{
    private CANSparkMax frontConveyor;
    private CANSparkMax backConveyor;
    private DigitalInput breakBeam;

    public VerticalConveyorSubsystem () {
        frontConveyor = new CANSparkMax(VerticalConveyorConstants.kFrontConveyorPort, MotorType.kBrushless);
        backConveyor = new CANSparkMax(VerticalConveyorConstants.kBackConveyorPort, MotorType.kBrushless);

        DigitalInput breakBeam = new DigitalInput(VerticalConveyorConstants.kVerticalIRBeamPort);
    }

    public void setVerticalSpeed(double percentPower) {
        frontConveyor.set(percentPower);
        backConveyor.set(percentPower);
    }

    public void shutdown() {
        frontConveyor.set(0);
        backConveyor.set(0);
    }

    public boolean isVerticalBeamBroken() {
        return !breakBeam.get();
    }

}
