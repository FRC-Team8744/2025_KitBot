// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;


public class CANDriveSubsystem extends SubsystemBase {
  
  private Lightbar Bar_bobik;

  private final SparkMax leftLeader;
   private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private RelativeEncoder m_EncoderLeft;
  private RelativeEncoder m_EncoderRight; 
  private final DifferentialDrive drive;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private SparkClosedLoopController rightPid;
  private SparkClosedLoopController leftPid;


  private PigeonIMU gyro; 
 // Create Field2d for robot and trajectory visualizations.
  public Field2d m_field;
  private final DifferentialDriveOdometry m_odometry;

  public CANDriveSubsystem() {

    // PID coefficients
    kP = 0.005; //0.00002; //1e-4; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.23; //0.00017; //0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    
  // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);
    Bar_bobik = new Lightbar();

    // create brushed motors for drive
    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless); 
    rightPid = rightLeader.getClosedLoopController();
    leftPid = leftLeader.getClosedLoopController();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    gyro = new PigeonIMU(13);
   

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
//    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    // config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);

    config.encoder
      .positionConversionFactor(DriveConstants.kEncoderDistancePerRevolution)
      .velocityConversionFactor(DriveConstants.kEncoderDistancePerRevolution / 60.0); //converting meters per second

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP, ClosedLoopSlot.kSlot0)
      .i(kI, ClosedLoopSlot.kSlot0)
      .d(kD, ClosedLoopSlot.kSlot0)
      .p(kP, ClosedLoopSlot.kSlot1)
      .i(kI, ClosedLoopSlot.kSlot1)
      .d(kD, ClosedLoopSlot.kSlot1)
      .velocityFF(kFF, ClosedLoopSlot.kSlot1)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    
    config.closedLoop.maxMotion
      .maxVelocity(1000, ClosedLoopSlot.kSlot0)
      .maxAcceleration(100000, ClosedLoopSlot.kSlot0)
      .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0)

      .maxVelocity(1000, ClosedLoopSlot.kSlot1)
      .maxAcceleration(100000, ClosedLoopSlot.kSlot1)
      .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    // Set configuration to follow leader and then apply it to corresponding
     //follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    config.inverted(true);
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(false);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_EncoderLeft = leftLeader.getEncoder();
    m_EncoderRight = rightLeader.getEncoder();
    m_EncoderLeft.setPosition(0.0);
    m_EncoderRight.setPosition(0.0);
    gyro.setYaw(0.0);  

    double StartX = 8.016;
    double StartY = 1.35;
    // m_odometry.resetPosition(null, null, null, null);
    // m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()), getEncoderMeters(m_EncoderLeft), getEncoderMeters(m_EncoderRight), new Pose2d(StartX, StartY, new Rotation2d(Math.PI)));
  }

  @Override
  public void periodic() {
    // double setPoint = m_stick.getY()*maxRPM;
    // maxPid.setReference(setPoint, SparkMax.ControlType.kVelocity);

    // SmartDashboard.putNumber("SetPoint", setPoint);
    
    SmartDashboard.putNumber("Velocity", m_EncoderLeft.getVelocity());
    SmartDashboard.putNumber("Velocity", m_EncoderRight.getVelocity());
    
    
    SmartDashboard.putNumber("Gyro Z", gyro.getYaw());
  
    SmartDashboard.putNumber("EncoderLeft", getEncoderMeters(m_EncoderLeft));
    SmartDashboard.putNumber("EncoderRight", getEncoderMeters(m_EncoderRight));
    // SmartDashboard.putNumber("xValue", RobotContainer.driverController.getLeftY());
    SmartDashboard.putNumber("Heading", getHeading());

    m_odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), getEncoderMeters(m_EncoderLeft), getEncoderMeters(m_EncoderRight));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    
  }

  private double getEncoderMeters(RelativeEncoder enc){
    return enc.getPosition();
  }
  
  public void driveVelocity(double xSpeed, double zRotation) {
  
    double leftSpeed = xSpeed - zRotation;
    double rightSpeed = xSpeed + zRotation;

    rightPid.setReference(rightSpeed, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    leftPid.setReference(leftSpeed, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);

    drive.feed();

    SmartDashboard.putNumber("rightSpeed", rightSpeed);
    SmartDashboard.putNumber("leftSpeed", leftSpeed);
    SmartDashboard.putNumber("rightVelocity", m_EncoderRight.getVelocity());
    SmartDashboard.putNumber("LeftVelocity", m_EncoderLeft.getVelocity());
  }

    // sets the speed of the drive motors
    public void driveArcade(double xSpeed, double zRotation) {
      drive.arcadeDrive(xSpeed, zRotation);
    }
  
    // sets the speed of the drive motors
    public void driveArcade(double xSpeed, double zRotation, boolean sqr) {
      drive.arcadeDrive(xSpeed, zRotation, sqr);
    }
  
    public double getHeading(){
      return Math.IEEEremainder(gyro.getYaw(), 360);
    }
  // // Command to drive the robot with joystick inputs
  // public Command driveArcade(
  //     CANDriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
  //   return Commands.run(
  //       () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
  // }
}
