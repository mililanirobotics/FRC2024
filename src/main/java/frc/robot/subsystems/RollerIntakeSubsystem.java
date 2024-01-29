package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.RollerIntakeConstants;


public class RollerIntakeSubsystem extends SubsystemBase {
    private CANSparkMax lowerRoller;
    private CANSparkMax upperRoller;

    public RollerIntakeSubsystem() {
      lowerRoller = new CANSparkMax(RollerIntakeConstants.kLowerRollerPort, MotorType.kBrushless);
      upperRoller = new CANSparkMax(RollerIntakeConstants.kUpperRollerPort, MotorType.kBrushless);
      lowerRoller.setInverted(RollerIntakeConstants.kLowerRollerReverse);
      upperRoller.setInverted(RollerIntakeConstants.kUpperRollerReverse);
    }

    public void setIntakeSpeed(double percentPower) {
      lowerRoller.set(percentPower);
      upperRoller.set(percentPower);
    }

    public void shutdown() {
      lowerRoller.set(0);
      upperRoller.set(0);
    }

    @Override
    public void periodic() {
      
    }
}