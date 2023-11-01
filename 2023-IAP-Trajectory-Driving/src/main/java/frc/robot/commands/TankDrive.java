// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  public DriveTrain dt; //Creates a new DriveTrain variable named dt
  public Joystick joy; //Creates a new Joystick variable named joy

  /** Creates a new TankDrive. */
  public TankDrive(DriveTrain dt, Joystick j) { //Moves DriveTrain dt and Joystick j to the TankDrive class
    this.dt = dt; //Sets variable dt = to dt
    this.joy = j; //Sets variable joy = to j

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.tankDrive(0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPowerRaw = joy.getRawAxis(1); //Gets joystick axis 1

    double rightPowerRaw = joy.getRawAxis(5); //Gets joystick axis 5

    dt.tankDrive(leftPowerRaw*-0.3, rightPowerRaw*-0.3); //Converts the axes into a speed by scaling by 0.7
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0); //Sets the speed to 0.0 since the command is ended or interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return false;
  }
}
