// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class EncoderDrive extends CommandBase {
  DriveTrain dt;
  double setPoint;
  /** Creates a new EncoderDrive. */
  public EncoderDrive(DriveTrain dt, double setPoint) {  //This is a constructor
    this.dt = dt;
    this.setPoint = setPoint;
    addRequirements (dt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    dt.tankDrive(0,0);
  } 


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  SmartDashboard.putNumber("meters", dt.getMeters());
  dt.tankDrive(0.3, 0.3);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0,0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(dt.getMeters() >= setPoint) {
      return true;
    }
    return false;
  }
}
