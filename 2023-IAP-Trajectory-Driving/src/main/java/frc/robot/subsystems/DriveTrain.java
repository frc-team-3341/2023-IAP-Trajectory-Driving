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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon; 
  private final WPI_TalonSRX rightDriveTalon;
  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;

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

    leftDriveTalon.setInverted(true); //This makes sure the leftDriveTalon is inverted in perspective to the rightDriveTalon so it can drive forward and backward properly
    rightDriveTalon.setInverted(false); //This makes sure the rightDriveTalon is inverted in perspective to the leftDriveTalon so it can drive forward and backward properly

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

  m_Field = new Field2d();
  m_Field.setRobotPose(new Pose2d(0,0,new Rotation2d(0)));



  drive = new DifferentialDrive(rightDriveTalon, leftDriveTalon);

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
  public double getAngle(){ //Gets the robot's current angle
    return -navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());



  }
  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(simLeftVoltage, simRightVoltage);
    m_driveSim.update(0.02);
    m_Field.setRobotPose(m_driveSim.getPose());
    SmartDashboard.putData("Field", m_Field);
    

  }
}