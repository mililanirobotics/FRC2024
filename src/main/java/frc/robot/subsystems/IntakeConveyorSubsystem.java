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
  //IR sensors
  // private DigitalInput startIntakeConveyorSensor;
  // private DigitalInput stopIntakeConveyorSensor;
  //
  private boolean inScorer;

  //constructor
  public IntakeConveyorSubsystem() {
    //initializing motor controllers
    leftConveyorTopRoller = new CANSparkMax(ConveyorIntakeConstants.kLeftConveyorTopRollerPort, MotorType.kBrushless);
    rightConveyorBottomRoller = new CANSparkMax(ConveyorIntakeConstants.kRightConveyorBottomRollerPort, MotorType.kBrushless);
    //setting directionality 
    leftConveyorTopRoller.setInverted(ConveyorIntakeConstants.kLeftConveyorTopRollerReverse);
    rightConveyorBottomRoller.setInverted(ConveyorIntakeConstants.kRightConveyorBottomRollerReverse);
    //IR sensors
    // startIntakeConveyorSensor = new DigitalInput(ConveyorIntakeConstants.kStartIntakeConveyorSensorPort);
    // stopIntakeConveyorSensor = new DigitalInput(ConveyorIntakeConstants.kStopIntakeConveyorSensorPort);
    //
    inScorer = false;
  }

  // /**
  //  * Returns whether or not the IR sensor beam on the bottom of the robot has been broken by the note
  //  * @return The state of the IR sensor
  //  */
  // public boolean getStartSensorReading() {
  //   return startIntakeConveyorSensor.get();
  // }

  // /**
  //  * Returns whether or not the IR sensor beam at the top of the conveyor has been broken by the note
  //  * @return The state of the IR sensor
  //  */
  // public boolean getStopSensorReading() {
  //   return stopIntakeConveyorSensor.get();
  // }

  /**
   * Returns whether the note has transitioned into the scorer
   * @return If the note is in the scorer or not
   */
  public boolean isInScorer() {
    return inScorer;
  }

  /**
   * Updates the value of the boolean inScorer based on the input
   * @param inScorer Whether the note is in the scorer
   */
  public void setInScorer(boolean inScorer) {
    this.inScorer = inScorer;
  }

  /**
   * Sets the speed of the conveyor based on the inputed speed
   * @param percentPower The requested speed of the motors
   */
  public void setSpeeds(double percentPower) {
    leftConveyorTopRoller.set(percentPower);
    rightConveyorBottomRoller.set(percentPower);
  }

  /**
   * Sets the power of the conveyor motors to 0
   */
  public void shutdown() {
      leftConveyorTopRoller.set(0);
      rightConveyorBottomRoller.set(0);
  }

  @Override
  public void periodic() {
    //prints the state of the IR sensors on Smartdashboard
    // SmartDashboard.putBoolean("Start Sensor Triggered", getStartSensorReading());
    // SmartDashboard.putBoolean("Stop Sensor Triggered", getStopSensorReading());
  }
}