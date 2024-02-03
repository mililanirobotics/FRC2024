package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//general imports
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//constants
import frc.robot.Constants.ConveyorIntakeConstants;

public class IntakeConveyorSubsystem extends SubsystemBase {
    //motor controllers
    private CANSparkMax leftConveyorTopRoller;
    private CANSparkMax rightConveyorBottomRoller;
    //IR sensor
    private DigitalInput conveyorIRSensor = new DigitalInput(ConveyorIntakeConstants.kConveyorIRSensorPort);

    //constructor
    public IntakeConveyorSubsystem() {
      //initializing motor controllers
      leftConveyorTopRoller = new CANSparkMax(ConveyorIntakeConstants.kLeftConveyorTopRollerPort, MotorType.kBrushless);
      rightConveyorBottomRoller = new CANSparkMax(ConveyorIntakeConstants.kRightConveyorPort, MotorType.kBrushless);
      //setting directionality 
      leftConveyorTopRoller.setInverted(ConveyorIntakeConstants.kLeftConveyorReverse);
      rightConveyorBottomRoller.setInverted(ConveyorIntakeConstants.kRightConveyorReverse);
    }

    /**
     * Returns whether or not the IR sensor beam has been broken by the note
     * @return The state of the IR sensor
     */
    public boolean isVerticalBeamBroken() {
      return conveyorIRSensor.get();
    }

    /**
     * Sets the speed of the conveyor based on the inputed speed
     * @param percentPower The requested speed of the motors
     */
    public void setConveyorSpeed(double percentPower) {
      leftConveyor.set(percentPower);
      rightConveyor.set(percentPower);
    }

    /**
     * Sets the power of the conveyor motors to 0
     */
    public void shutdown() {
        leftConveyor.set(0);
        rightConveyor.set(0);
    }

    @Override
    public void periodic() {
      //prints the state of the conveyor IR sensor on Smartdashboard
      SmartDashboard.putBoolean("Conveyor Sensor Triggered", isVerticalBeamBroken());
    }
}