// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon; 
  private final WPI_TalonSRX rightDriveTalon;
  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;
  private final DifferentialDriveOdometry odometry;

  private AHRS navx = new AHRS(SPI.Port.kMXP); //This is creating a new object within the AHRS class called navx

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain"); //This titles the variable DTTab as DriveTrain.
  private double simLeftVoltage;
  private double simRightVoltage;

  private DifferentialDrivetrainSim m_driveSim;
  private DifferentialDrive drive;
  private Field2d m_Field;
  
  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort); //This sets up the leftDriveTalon
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort); //This sets up the rightDriveTalon
    leftDriveSim = leftDriveTalon.getSimCollection();
    rightDriveSim = rightDriveTalon.getSimCollection();
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast); //This makes sure that the leftDriveTalon is set to neutral
    rightDriveTalon.setNeutralMode(NeutralMode.Coast); //This makes sure that the rightDriveTalon is set to neutral

    leftDriveTalon.setInverted(false); //This makes sure the leftDriveTalon is inverted in perspective to the rightDriveTalon so it can drive forward and backward properly
    rightDriveTalon.setInverted(true); //This makes sure the rightDriveTalon is inverted in perspective to the leftDriveTalon so it can drive forward and backward properly

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault(); //resets leftDriveTalon
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //This sets up the encoder to track rotation of the motor
    rightDriveTalon.configFactoryDefault(); //resets rightDriveTalon
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // Create the simulation model of our drivetrain.
 m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
  7.29,                    // 7.29:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  60.0,                    // The mass of the robot is 60 kg.
  Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  0.7112,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

int x = 4;
int y = 4;

  m_driveSim.setPose(new Pose2d(x,y, new Rotation2d(0)));
  m_Field = new Field2d();
  m_Field.setRobotPose(new Pose2d(x,y,new Rotation2d(0)));



  drive = new DifferentialDrive(rightDriveTalon, leftDriveTalon);
  odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {  //This will drive the robot with a certain speed
    rightDriveTalon.set(rightSpeed); 
    leftDriveTalon.set(leftSpeed);
    simLeftVoltage = leftSpeed*12.0;
    simRightVoltage = rightSpeed*12.0;
    drive.feed();
    
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10); //Sets the sensor position of the leftDriveTalon to 0, 0, 10
    rightDriveTalon.setSelectedSensorPosition(0,0,10); //Sets the sensor position of the rightDriveTalon to 0, 0, 10
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
 
  public double getMeters(){
    return (Units.inchesToMeters(6)*Math.PI /4096*getTicks());
  }  
  public double metersToTicks(double positionMeters){
    return (positionMeters / (0.1524 * Math.PI)*4096);
  }
  public double getAngle(){ //Gets the robot's current angle
    return -navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }
/**
   * Returns displacement of left side of chassis.
   * 
   * @return the displacement in meters (m)
   */
  public double getLeftDistance() {
    return leftDriveTalon.getSelectedSensorPosition() / Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns displacement of right side of chassis.
   * 
   * @return the displacement in meters (m)
   */
  public double getRightDistance() {

    return rightDriveTalon.getSelectedSensorPosition() / Constants.DriveToLineConstants.ticksToMeters;
  }
  public double getLeftSpeed() {
    return (leftDriveTalon.getSelectedSensorVelocity() * 10.0) / Constants.DriveToLineConstants.ticksToMeters;
  }
  public double getRightSpeed() {
    return (leftDriveTalon.getSelectedSensorVelocity() * 10.0) / Constants.DriveToLineConstants.ticksToMeters;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
    SmartDashboard.putNumber("Ticks", getTicks());
  }
  @Override
  public void simulationPeriodic() { 
    m_driveSim.setInputs(simLeftVoltage, simRightVoltage);
    m_driveSim.update(0.02);
    m_Field.setRobotPose(m_driveSim.getPose());
    SmartDashboard.putData("Field", m_Field);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(m_driveSim.getHeading().getDegrees());
    leftDriveTalon.setSelectedSensorPosition(metersToTicks(m_driveSim.getLeftPositionMeters()),0,10);
    rightDriveTalon.setSelectedSensorPosition(metersToTicks(m_driveSim.getRightPositionMeters()),0,10);

    //Updates the Quadrature for leftDriveSim

    leftDriveSim.setQuadratureRawPosition( 
        distanceToNativeUnits(
            -m_driveSim.getLeftPositionMeters())); //Gets the position for the left motor in meters
    leftDriveSim.setQuadratureVelocity(
        velocityToNativeUnits(
            -m_driveSim.getLeftVelocityMetersPerSecond())); //Gets the velocity for the left motor in meters per second
      

    // Update Quadrature for Right
    // Have to flip, to match phase of real encoder
    // Left wheel goes CCW, Right goes CW for forward by default

    rightDriveSim.setQuadratureRawPosition( 
        distanceToNativeUnits(
            m_driveSim.getRightPositionMeters())); //Gets the position for the right motor in meters
    rightDriveSim.setQuadratureVelocity(
        velocityToNativeUnits(
            m_driveSim.getRightVelocityMetersPerSecond())); //Gets the velocity for the right motor in meters per second
    
  

    SmartDashboard.putNumber("Heading", m_driveSim.getHeading().getDegrees());

    SmartDashboard.putNumber("LeftPosition", getLeftDistance());
    SmartDashboard.putNumber("RightPosition", getRightDistance());
    SmartDashboard.putNumber("LeftVel", getLeftSpeed());
    SmartDashboard.putNumber("RightVel", getRightSpeed());

    // Turn rate returns 0 in sim, same in real life?
    // Turn rate is never used
    SmartDashboard.putNumber("TurnRate", getTurnRate());
    SmartDashboard.putNumber("SimAng", angle.get());
    if (Robot.isSimulation()) {
      odometry.update(navx.getRotation2d().unaryMinus(), getLeftDistance(), getRightDistance());
    }

  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }




  //
  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters
        / (Math.PI * Units.inchesToMeters(Constants.DriveToLineConstants.wheelDiameterInInches)); //Converts how many inches the motor rotated to meters
    double motorRotations = wheelRotations * 1.0; //Sets double motorRotations equal to double wheelRotations
    int sensorCounts = (int) (motorRotations * 4096.0); //Sets int sensorCounts equal to motorRotations * 4096.0, and then changed into an int
    return sensorCounts; //returns the value of the variable sensorCounts
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond) {
    // Previous mistake: multiply this by 2
    // Consequences: had to set the constant to 0.5 less
    // Now it works without the 2
    double wheelRotationsPerSecond = velocityMetersPerSecond
        / (Math.PI * Units.inchesToMeters(Constants.DriveToLineConstants.wheelDiameterInInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }
  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / 4096.0;
    double wheelRotations = motorRotations / 1.0;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(6.0));
    return positionMeters;
}
public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }
  public double getTurnRate() {
    return -navx.getRate();
  }
}

