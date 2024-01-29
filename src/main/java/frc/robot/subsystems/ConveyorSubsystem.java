package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
    private CANSparkMax leftConveyor;
    private CANSparkMax rightConveyor;

    public ConveyorSubsystem() {
      leftConveyor = new CANSparkMax(ConveyorConstants.kLeftConveyorPort, MotorType.kBrushless);
      rightConveyor = new CANSparkMax(ConveyorConstants.kRightConveyorPort, MotorType.kBrushless);

      leftConveyor.setInverted(ConveyorConstants.kLeftConveyorReverse);
      rightConveyor.setInverted(ConveyorConstants.kRightConveyorReverse);
    }

    public void setConveyorSpeed(double percentPower) {
      leftConveyor.set(percentPower);
      rightConveyor.set(percentPower);
    }

    public void shutdown() {
        leftConveyor.set(0);
        rightConveyor.set(0);
    }

    @Override
    public void periodic() {
    }
}