// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTurnCCW extends CommandBase {
  DriveTrain dt;
  double setPointAngle;
  PIDController pid = new PIDController(0.3/90, 0.005, 0);
  int motorSign;

  
  /** Creates a new PIDTurnCCW. */
  public PIDTurnCCW(DriveTrain dt, double setPointAngle) { 
    this.dt = dt; //Sets variable dt = to dt
    this.setPointAngle = setPointAngle; //Sets setPointAngle = to setPointAngle
    addRequirements(dt);
    pid.setTolerance(5.0); //Tells the robot how much it can overshoot or undershoot by
    if(setPointAngle > 0 ) { //Counterclockwise turn
      motorSign = 1;
    }
    else {
      motorSign = -1; //Clockwise turn

    }
    }
    // Use addRequirements() here to declare subsystem dependencies.
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     dt.resetNavx(); //Resets the Navx gyroscope
     dt.tankDrive(0, 0); //Sets the motor power to 0
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(dt.getAngle(), setPointAngle); //Gets the current angle and calculates how far off it is from the final angle
    dt.tankDrive(-output*motorSign, output*motorSign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0,0); //Sets motor power to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
    
     //If this is true, the robot is at the setPointAngle
  }
}
