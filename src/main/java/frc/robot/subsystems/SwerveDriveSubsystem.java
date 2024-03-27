package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
//general imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
//path planner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
    //Swerve modules
    private SwerveModule leftFrontModule;
    private SwerveModule rightFrontModule;
    private SwerveModule leftBackModule;
    private SwerveModule rightBackModule;
    private CANSparkMax tempSpark1;
    private CANSparkMax tempSpark2;
    //array of modules; used to get states later
    private SwerveModule[] modules;
    //Navx and odometry
    private AHRS navX;
    private SwerveDriveOdometry odometry;
    //Field2d and simulation
    private Field2d field;
    private PowerDistribution pdh;
    //position translational 
    private GenericEntry leftFrontTranPosWidget;
    private GenericEntry leftBackTranPosWidget;
    private GenericEntry rightFrontTranPosWidget;
    private GenericEntry rightBackTranPosWidget;
    //velocity translational
    private GenericEntry leftFrontTranVelWidget;
    private GenericEntry leftBackTranVelWidget;
    private GenericEntry rightFrontTranVelWidget;
    private GenericEntry rightBackTranVelWidget;
    //position rotational 
    private GenericEntry leftFrontRotPosWidget;
    private GenericEntry leftBackRotPosWidget;
    private GenericEntry rightFrontRotPosWidget;
    private GenericEntry rightBackRotPosWidget;
    //velocity rotational
    private GenericEntry leftFrontRotVelWidget;
    private GenericEntry leftBackRotVelWidget;
    private GenericEntry rightFrontRotVelWidget;
    private GenericEntry rightBackRotVelWidget;  
    //pdh widgets
    private GenericEntry leftFrontTranAmps;
    private GenericEntry leftBackTranAmps;
    private GenericEntry rightFrontTranAmps;
    private GenericEntry rightBackTranAmps;

    //gyro data
    private GenericEntry gyroData;

    //constructor
    public SwerveDriveSubsystem(ShuffleboardTab testTranPos, ShuffleboardTab testTranVel, ShuffleboardTab testRotPos, 
                                 ShuffleboardTab testRotVel, ShuffleboardTab testPos, ShuffleboardTab testGyroData) {

        //temporary Sparkmaxes for testing
        tempSpark1 = new CANSparkMax(10, MotorType.kBrushless);
        tempSpark2 = new CANSparkMax(11, MotorType.kBrushless);
        
        //Front Left Module Initializing
        leftFrontModule = new SwerveModule(
            SwerveModuleConstants.kLeftFrontWheelPort, 
            SwerveModuleConstants.kLeftFrontRotationPort, 
            SwerveModuleConstants.kLeftFrontDriveReversed, 
            SwerveModuleConstants.kLeftFrontRotationReversed, 
            SwerveModuleConstants.kLeftFrontCANCoderPort, 
            SwerveModuleConstants.kLeftFrontCANCoderOffset, 
            SwerveModuleConstants.kLeftFrontCANCoderReversed,
            SwerveModuleConstants.kLeftDriveCurrentLimit
        );

        //Front Right Module Initializing
        rightFrontModule = new SwerveModule(
            SwerveModuleConstants.kRightFrontWheelPort, 
            SwerveModuleConstants.kRightFrontRotationPort, 
            SwerveModuleConstants.kRightFrontDriveReversed, 
            SwerveModuleConstants.kRightFrontRotationReversed, 
            SwerveModuleConstants.kRightFrontCANCoderPort, 
            SwerveModuleConstants.kRightFrontCANCoderOffset, 
            SwerveModuleConstants.kRightFrontCANCoderReversed,
            SwerveModuleConstants.kRightDriveCurrentLimit
        );

        //Back Left Module Initializing
        leftBackModule = new SwerveModule(
            SwerveModuleConstants.kLeftBackWheelPort, 
            SwerveModuleConstants.kLeftBackRotationPort, 
            SwerveModuleConstants.kLeftBackDriveReversed, 
            SwerveModuleConstants.kLeftBackRotationReversed, 
            SwerveModuleConstants.kLeftBackCANCoderPort, 
            SwerveModuleConstants.kLeftBackCANCoderOffset, 
            SwerveModuleConstants.kLeftBackCANCoderReversed,
            SwerveModuleConstants.kLeftDriveCurrentLimit
        );

        //Back Right Module Initializing
        rightBackModule = new SwerveModule(
            SwerveModuleConstants.kRightBackWheelPort, 
            SwerveModuleConstants.kRightBackRotationPort, 
            SwerveModuleConstants.kRightBackDriveReversed, 
            SwerveModuleConstants.kRightBackRotationReversed, 
            SwerveModuleConstants.kRightBackCANCoderPort, 
            SwerveModuleConstants.kRightBackCANCoderOffset, 
            SwerveModuleConstants.kRightBackCANCoderReversed,
            SwerveModuleConstants.kRightDriveCurrentLimit
        );

        //adding modules to the array
        modules = new SwerveModule[] {
            leftFrontModule, rightFrontModule, leftBackModule, rightBackModule
        };

        //initializing and resetting Navx
        navX = new AHRS(SPI.Port.kMXP);
        navX.enableLogging(true);
        navX.reset();

        //initializing odometry that uses continuous 360 degree input
        odometry = new SwerveDriveOdometry(
            SwerveModuleConstants.kinematics, 
            navX.getRotation2d(),
            getModulePosition()
        );

        //reseting members
        zeroOutGyro();
        resetOdometry(getPose());
        resetOdometry(new Pose2d());

        //tuning PID 
        DriveConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);
        DriveConstants.thetaController.setTolerance(Units.degreesToRadians(10));

        //initializing AutoBuilder to create path planner autopaths
        //flips the created autopath if on the Red Alliance
        initAutoBuilder();

        //Path Planner logging
        field = new Field2d();

        //Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback(
            (pose) -> {
                field.setRobotPose(pose);
            }
        );

        //Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
            field.getObject("target pose").setPose(pose);
        }
        );

        //Logging callback for the active path, this is sent as a list of poses 
        PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
            field.getObject("path").setPoses(poses);
        }
        );

        testPos.add("Field", field).withSize(5, 2);

        //adding translational position widgets
        leftFrontTranPosWidget = testTranPos.add("left_front_tran_pos", 0).withSize(2, 1).getEntry();
        leftBackTranPosWidget = testTranPos.add("left_back_tran_pos", 0).withSize(2, 1).getEntry();
        rightFrontTranPosWidget = testTranPos.add("right_front_tran_pos", 0).withSize(2, 1).getEntry();
        rightBackTranPosWidget = testTranPos.add("right_back_tran_pos", 0).withSize(2, 1).getEntry();
        //adding translational velocity widgets
        leftFrontTranVelWidget = testTranVel.add("left_front_tran_vel", 0).withSize(2, 1).getEntry();
        leftBackTranVelWidget = testTranVel.add("left_back_tran_vel", 0).withSize(2, 1).getEntry();
        rightFrontTranVelWidget = testTranVel.add("right_front_tran_vel", 0).withSize(2, 1).getEntry();
        rightBackTranVelWidget = testTranVel.add("right_back_tran_vel", 0).withSize(2, 1).getEntry();
        //adding rotational position widgets
        leftFrontRotPosWidget = testRotPos.add("left_front_rot_pos", 0).withSize(2, 1).getEntry();
        leftBackRotPosWidget = testRotPos.add("left_back_rot_pos", 0).withSize(2, 1).getEntry();
        rightFrontRotPosWidget = testRotPos.add("right_front_rot_pos", 0).withSize(2, 1).getEntry();
        rightBackRotPosWidget = testRotPos.add("right_back_rot_pos", 0).withSize(2, 1).getEntry();
        //adding rotational velocity widgets
        leftFrontRotVelWidget = testRotVel.add("left_front_rot_vel", 0).withSize(2, 1).getEntry();
        leftBackRotVelWidget = testRotVel.add("left_back_rot_vel", 0).withSize(2, 1).getEntry();
        rightFrontRotVelWidget = testRotVel.add("right_front_rot_vel", 0).withSize(2, 1).getEntry();
        rightBackRotVelWidget = testRotVel.add("right_back_rot_vel", 0).withSize(2, 1).getEntry();
        //adding gyro widget
        gyroData = testGyroData.add("gyro_data", getYaw()).withSize(2, 1).getEntry();

        pdh = new PowerDistribution(1, ModuleType.kRev);

        leftFrontTranAmps = testTranPos.add("left_front_tran_amps", 0).withSize(2, 1).getEntry();
        leftBackTranAmps = testTranPos.add("left_back_tran_amps", 0).withSize(2, 1).getEntry();
        rightFrontTranAmps = testTranPos.add("right_front_tran_amps", 0).withSize(2, 1).getEntry();
        rightBackTranAmps = testTranPos.add("right_back_tran_amps", 0).withSize(2, 1).getEntry();
    }

    //=========================================================================== 
    // gyro and accelorometer methods
    //===========================================================================

    /**
     * Resets the current angle of the gyro to 0. 
     * Tells the driver that the gyro is connected via a print statement
    */
    public void zeroOutGyro() {
        System.out.println("Gyro Connected: "+navX.isConnected());
        navX.reset();
    }

    /**
     * Gets the current yaw angle from the navx gyro
     * @return The yaw value recorded
     */
    public double getYaw() {
        return navX.getYaw();
    }

    /**
     * Gets the current pitch angle from the navx gyro
     * @return The pitch value recorded
     */
    public double getPitch() {
        return navX.getPitch();
    }

    /**
     * Gets the current roll angle from the navx gyro
     * @return The roll value recorded
     */
    public double getRoll() {
        return navX.getRoll();
    }

    /**
     * Returns the angle of the gyro based on its current rotation 2D object (continuous)
     * @return The angle reported by the gyro
     */
    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    /**
     * Returns the angular velocity of the rotation motors based on the NavX's reported rotation rate (deg/s)
     * @return
     */
    public double getRotationRate() {
        return -navX.getRate();
    }

    /**
     * Returns a Rotation2D object based on the NavX's angle
     * @return The rotation 2D object
     */
    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }

    /**
     * Returns a Rotation2d object from a continuous 0-360 degree rotation
     * @return The continuous degree Rotation2d object
     */
    public Rotation2d getRotation2dDegContinuous() {
        return Rotation2d.fromDegrees(getDegrees());
    }

    /**
     * Returns a Rotation2d object from a continuous 0-2π degree rotation
     * @return The continuous radian Rotation2d object
     */
    public Rotation2d getRotation2dRadContinuous() {
        return Rotation2d.fromRadians(getRad());
    }

    /**
     * Returns a Rotation2d object from a -180 to 180 degree rotation
     * @return The degree Rotation2d object
     */
    public Rotation2d getRotation2dDeg() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Returns a Rotation2d object from -π to π radian rotation
     * @return The radian Rotation2d object
     */
    public Rotation2d getRotation2dRad() {
        return Rotation2d.fromRadians(Units.degreesToRadians(getYaw()));
    }

    /**
     * Returns the yaw value of the Navx assuming CCW pos and continuous (in degrees)
     * @return The adjusted degrees
     */
    public double getDegrees() {
        double rawDegrees = -getYaw();
        return rawDegrees < 0 ? rawDegrees + 360 : rawDegrees;
    }

    /**
     * Returns the yaw value of the Navx assuming CCW pos and continuous (in radians)
     * @return The adjusted radians
     */
    public double getRad() {
        double rad = Units.degreesToRadians(-getYaw());
        return rad < 0 ? rad + 2 * Math.PI : rad;
    }

    public double getAngleReference() {
        return leftFrontModule.getRotationPosition();
    }

    //=========================================================================== 
    // drive methods
    //===========================================================================

    /**
     * Sets the desired states for the swere modules; modules will automatically go to the desired states
     * @param desiredStates The desired states from the ChassisSpeeds object
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //sets drive constants to the states
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kDriveMaxMetersPerSecond);
        //sets the states
        leftFrontModule.setSwerveState(desiredStates[0], false);
        rightFrontModule.setSwerveState(desiredStates[1], false);
        leftBackModule.setSwerveState(desiredStates[2], false);
        rightBackModule.setSwerveState(desiredStates[3], false);
    }

    /**
     * Returns the current swerve module states
     * @return An array of the current swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(int i = 0; i < states.length; i++) {
            states[i] = modules[i].getModuleState();
        }

        return states;
    }

    /**
     * Returns an array of all the module states
     * @return The array of module states
     */
    public SwerveModule[] getModules() {
        return modules;
    }

    /**
    * Returns the current position of the module (displacement)
    * @return The current displacement of the module
    */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(
                leftFrontModule.getDrivePosition(), Rotation2d.fromRadians(leftFrontModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                rightFrontModule.getDrivePosition(), Rotation2d.fromRadians(rightFrontModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                leftBackModule.getDrivePosition(), Rotation2d.fromRadians(leftBackModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                rightBackModule.getDrivePosition(), Rotation2d.fromRadians(rightBackModule.getRotationPosition())
            )
        };
    }

    /**
     * Returns a robot oriented ChassisSpeed object
     * @return The robot oriented ChassisSpeed object
     */
    public ChassisSpeeds getSpeeds() {
        return SwerveModuleConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the swerve states to robot relative control
     * @param robotRelativeSpeed The robot relative ChassisSpeed object
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeed) {
        setModuleStates(SwerveModuleConstants.kinematics.toSwerveModuleStates(robotRelativeSpeed));
    }

    /**
     * Sets the power of the conveyor motors to 0
     */
    public void shutdown() {
        leftFrontModule.shutdown();
        rightFrontModule.shutdown();
        leftBackModule.shutdown();
        rightBackModule.shutdown();
    }

    /**
     * Resets the encoders of all the modules
     * @param startT The translational defualt value of the drive encoder
     * @param startR The rotational default value of the rotating encoder 
     */
    public void resetEncoders(double startT, double startR) {
        leftFrontModule.resetEncoders(startT, startR);
        leftBackModule.resetEncoders(startT, startR);
        rightFrontModule.resetEncoders(startT, startR);
        rightBackModule.resetEncoders(startT, startR);
    }

    /**
     * Sets the mode of the modules to brake
     */
    public void setBrake() {
        leftFrontModule.setBrake();
        leftBackModule.setBrake();
        rightFrontModule.setBrake();
        rightBackModule.setBrake();
    }

    /**
     * Sets the mode of the modules to coast
     */
    public void setCoast() {
        leftFrontModule.setCoast();
        leftBackModule.setCoast();
        rightFrontModule.setCoast();
        rightBackModule.setCoast();
    }

    /**
     * Returns the average position reported by the four modules (meters)
     * @return The average translational position of the four modules
     */
    public double getAvgEncoderTran() {
        return (leftBackModule.getDrivePosition() + 
            leftFrontModule.getDrivePosition() + 
            rightBackModule.getDrivePosition() + 
            rightFrontModule.getDrivePosition() /
            4.0
        );
    }

    /**
     * Returns the average position reported by the four modules (radians)
     * @return The average rotational position of the four modules
     */
    public double getAvgEncoderRot() {
        return (leftBackModule.getRotationPosition() + 
            leftFrontModule.getRotationPosition() + 
            rightBackModule.getRotationPosition() + 
            rightFrontModule.getRotationPosition() /
            4.0
        );
    }

    //=========================================================================== 
    // misc methods
    //===========================================================================
    
    private void initAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            AutoConstants.pathFollowingConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) { 
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }
       
    /**
     * Returns the recorded odometry position of the robot on the field 
     * @return
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry object; sets its position based on the robot's current rotation and position
     * @param pose The current position of the robot
     */
    public void resetOdometry(Pose2d pose) {
        //resets encoders
        resetEncoders(pose.getTranslation().getNorm(), pose.getRotation().getRadians());

        //odometry 
        odometry.resetPosition(
            navX.getRotation2d(),
            getModulePosition(), 
            pose
        );
    }

    /**
     * Prints the CANCoder readings to SmartDashboard
     */
    public void getCANCoderReading() {
        SmartDashboard.putNumber("Left Front: ", leftFrontModule.getCANCoderReading());
        SmartDashboard.putNumber("Right Front: ", rightFrontModule.getCANCoderReading());
        SmartDashboard.putNumber("Left Back: ", leftBackModule.getCANCoderReading());
        SmartDashboard.putNumber("Right Back: ", rightBackModule.getCANCoderReading());
        SmartDashboard.updateValues();
    }

    public void printTestingData() {
        //translational position
        leftFrontTranPosWidget.setDouble(leftFrontModule.getDrivePosition());
        leftBackTranPosWidget.setDouble(leftBackModule.getDrivePosition());
        rightFrontTranPosWidget.setDouble(rightFrontModule.getDrivePosition());
        rightBackTranPosWidget.setDouble(rightBackModule.getDrivePosition());
        //amps
        leftFrontTranAmps.setDouble(pdh.getCurrent(SwerveModuleConstants.kLeftFrontWheelPort));
        leftBackTranAmps.setDouble(pdh.getCurrent(SwerveModuleConstants.kLeftBackWheelPort));
        rightFrontTranAmps.setDouble(pdh.getCurrent(SwerveModuleConstants.kRightFrontWheelPort));
        rightBackTranAmps.setDouble(pdh.getCurrent(1));
        //translational velocity
        leftFrontTranVelWidget.setDouble(leftFrontModule.getDriveVelocity());
        leftBackTranVelWidget.setDouble(leftBackModule.getDriveVelocity());
        rightFrontTranVelWidget.setDouble(rightFrontModule.getDriveVelocity());
        rightBackTranVelWidget.setDouble(rightBackModule.getDriveVelocity());
        //rotational position
        leftFrontRotPosWidget.setDouble(leftFrontModule.getRotationPosition());
        leftBackRotPosWidget.setDouble(leftBackModule.getRotationPosition());
        rightFrontRotPosWidget.setDouble(rightFrontModule.getRotationPosition());
        rightBackRotPosWidget.setDouble(rightBackModule.getRotationPosition());
        //rotational velocity
        leftFrontRotVelWidget.setDouble(leftFrontModule.getRotationVelocity());
        leftBackRotVelWidget.setDouble(leftBackModule.getRotationVelocity());
        rightFrontRotVelWidget.setDouble(rightFrontModule.getRotationVelocity());
        rightBackRotVelWidget.setDouble(rightBackModule.getRotationVelocity());
    }

    @Override
    public void periodic() {
        //updates odometry
        odometry.update(
            navX.getRotation2d(),
            getModulePosition()        
        );

        //updates the robot's pose on the Field2d widget
        field.setRobotPose(odometry.getPoseMeters());

        //prints the CANcoder readings on all 4 modules 
        printTestingData();
        getCANCoderReading();

        //path planner
        PathPlannerLogging.setLogCurrentPoseCallback(
            (pose) -> {
                field.setRobotPose(pose);
            }
        );

        //Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
            (pose) -> {
                field.getObject("target pose").setPose(pose);
            }
        );

        //Logging callback for the active path, this is sent as a list of poses 
        PathPlannerLogging.setLogActivePathCallback(
            (poses) -> {
                field.getObject("path").setPoses(poses);
            }
        );
    }
}
