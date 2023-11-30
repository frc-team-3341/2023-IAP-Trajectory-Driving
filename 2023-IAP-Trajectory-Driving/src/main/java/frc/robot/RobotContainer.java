// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.Delayed;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EncoderDrive;
import frc.robot.commands.PIDTurnCCW;
import frc.robot.commands.Ramsete;


public class RobotContainer {
  public static DriveTrain dt = new DriveTrain();
  public PIDTurnCCW pid = new PIDTurnCCW(dt, 90, false);
  public static Joystick j = new Joystick(0);
  public Ramsete ramsete = new Ramsete();
  public RobotContainer() {
    dt.setDefaultCommand(new TankDrive(dt, j));
    configureBindings();
   }
  




  private void configureBindings() {} 

  public Command getAutonomousCommand() {
    return ramsete;
     
      
      
      
      


    
  }
}
