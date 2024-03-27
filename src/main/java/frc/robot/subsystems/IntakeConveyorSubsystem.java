package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//general imports
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//constants
import frc.robot.Constants.IntakeConveyorConstants;

public class IntakeConveyorSubsystem extends SubsystemBase {
  //motor controllers
  private CANSparkMax greenRollerConveyor;
  private CANSparkMax blueRollerConveyor;
  //IR sensors
  private DigitalInput startIntakeConveyorSensor;
  private DigitalInput stopIntakeConveyorSensor;

  //constructor
  public IntakeConveyorSubsystem() {
    //initializing motor controllers
    greenRollerConveyor = new CANSparkMax(IntakeConveyorConstants.kGreenRollerConveyorPort, MotorType.kBrushless);
    blueRollerConveyor = new CANSparkMax(IntakeConveyorConstants.kBlueRollerConveyorPort, MotorType.kBrushless);
    //setting directionality 
    greenRollerConveyor.setInverted(IntakeConveyorConstants.kGreenRollerConveyorReverse);
    blueRollerConveyor.setInverted(IntakeConveyorConstants.kBlueRollerConveyorReverse);
    //IR sensors
    startIntakeConveyorSensor = new DigitalInput(IntakeConveyorConstants.kStartIntakeConveyorSensorPort);
    stopIntakeConveyorSensor = new DigitalInput(IntakeConveyorConstants.kStopIntakeConveyorSensorPort);
  
  }

  /**
   * Returns whether or not the IR sensor beam on the bottom of the robot has been broken by the note
   * @return The state of the IR sensor
   */
  public boolean getStartSensorReading() {
    return startIntakeConveyorSensor.get();
  }

  public boolean getStartSensorReadingReverse() {
    return !startIntakeConveyorSensor.get();
  }

  /**
   * Returns whether or not the IR sensor beam at the top of the conveyor has been broken by the note
   * @return The state of the IR sensor
   */
  public boolean getStopSensorReading() {
    return stopIntakeConveyorSensor.get();
  }

  /**
   * Sets the speed of the conveyor based on the inputed speed
   * @param percentPower The requested speed of the motors
   */
  public void setSpeeds(double percentPower) {
    greenRollerConveyor.set(percentPower);
    blueRollerConveyor.set(percentPower);
  }

  /**
   * Sets the power of the conveyor motors to 0
   */
  public void shutdown() {
      greenRollerConveyor.set(0);
      blueRollerConveyor.set(0);
  }

  @Override
  public void periodic() {
    //prints the state of the IR sensors on Smartdashboard
    SmartDashboard.putBoolean("Start Sensor Triggered", getStartSensorReading());
    SmartDashboard.putBoolean("Stop Sensor Triggered", getStopSensorReading());
  }
}