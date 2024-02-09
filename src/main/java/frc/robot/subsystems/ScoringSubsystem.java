package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//general imports
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//constants
import frc.robot.Constants.ScoringConstants;

public class ScoringSubsystem extends SubsystemBase {
  //motor controllers
  private CANSparkMax topLaunchRoller;
  private CANSparkMax bottomLaunchRoller;
  //IR sensor
  // private DigitalInput stopScoringSensor;

  //constructor
  public ScoringSubsystem() {
    //initializing motor controllers
    topLaunchRoller = new CANSparkMax(ScoringConstants.kUpperFlywheelPort, MotorType.kBrushless);
    bottomLaunchRoller = new CANSparkMax(ScoringConstants.kLowerFlywheelPort, MotorType.kBrushless);
    //setting directionality 
    topLaunchRoller.setInverted(ScoringConstants.kUpperFlywheelReverse);
    bottomLaunchRoller.setInverted(ScoringConstants.kLowerFlywheelReverse);
    //IR sensors
    // stopScoringSensor = new DigitalInput(ScoringConstants.kScoringSensorPort);
  }

  /**
   * Powers the scoring rollers
   * @param botPower The power of the bottom roller
   * @param topPower The power of the top roller
   */
  public void setSpeed(double botPower, double topPower) {
    topLaunchRoller.set(topPower);
    bottomLaunchRoller.set(botPower);
  }

  // /**
  //  * Returns whether or not the IR sensor beam at the end of the scoring payload has been broken by the note
  //  * @return The state of the IR sensor
  //  */
  // public boolean getStopSensorReading() {
  //   return stopScoringSensor.get();
  // }

  /**
   * Sets the power of the conveyor motors to 0
   */
  public void shutdown() {
    topLaunchRoller.set(0);
    bottomLaunchRoller.set(0);
  }

  @Override
  public void periodic() {
    //prints the state of the IR sensor on Smartdashboard
    // SmartDashboard.putBoolean("Scoring Sensor Triggered", getStopSensorReading());
  }
}