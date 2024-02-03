package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//general imports
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//constants
import frc.robot.Constants.RollerIntakeConstants;

public class IntakeRollerSubsystem extends SubsystemBase {
    //motor controllers
    private CANSparkMax topRoller;
    private CANSparkMax bottomRoller;
    //IR sensor
    private DigitalInput RollerIRSensor; 

    //constructor
    public IntakeRollerSubsystem() {
      //initializing motor controllers
      topRoller = new CANSparkMax(RollerIntakeConstants.kTopRollerPort, MotorType.kBrushless);
      bottomRoller = new CANSparkMax(RollerIntakeConstants.kBottomRollerPort, MotorType.kBrushless);

      RollerIRSensor = new DigitalInput(RollerIntakeConstants.kRollerIRSensorPort);
      //setting directionality 
      topRoller.setInverted(RollerIntakeConstants.kTopRollerReverse);
      bottomRoller.setInverted(RollerIntakeConstants.kBottomRollerReverse);
    }

    /**
     * Returns whether or not the IR sensor beam has been broken by the note
     * @return The state of the IR sensor
     */
    public boolean isRollerBeamBroken() {
      return RollerIRSensor.get();
    }

    /**
     * Sets the speed of the conveyor based on the inputed speed
     * @param percentPower The requested speed of the motors
     */
    public void setIntakeSpeed(double percentPower) {
      topRoller.set(percentPower);
      bottomRoller.set(percentPower);
    }

    /**
     * Sets the power of the conveyor motors to 0
     */
    public void shutdown() {
        topRoller.set(0);
        bottomRoller.set(0);
    }

    @Override
    public void periodic() {
      //prints the state of the conveyor IR sensor on Smartdashboard
      SmartDashboard.putBoolean("Conveyor Sensor Triggered", isRollerBeamBroken());
    }
}