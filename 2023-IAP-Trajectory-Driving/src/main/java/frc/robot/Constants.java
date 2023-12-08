// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainPorts {
        public static final int LeftDriveTalonPort = 2; 
        public static final int RightDriveTalonPort = 3;
        public static final int LeftDriveVictorPort = 4;
        public static final int RightDriveVictorPort = 5;
        public static final int wheelDiameterInInches = 6;
    }
    public static final class DriveToLineConstants {
        public static final double wheelDiameterInInches = 6.0;
        public static final double ticksToMeters = (1.0 / (wheelDiameterInInches * 0.0254 * Math.PI)) * 4096.0;
       
    }


    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;
        public static final double leftPowerRaw = 0.3;
        public static final double rightPowerRaw = 0.3;
        public static final int getMeters = 6;
        public static final double setTolerance = 5.0;
        public static final double ki = 0.002;
        public static final double kp = 0.3/90;
        public static final int setPointAngle = 90;
    }
    public static class SimConstants {
        public static final double kS = 0.22;
        public static final double kV = 2.98;
        public static final double kA = 0.2;

        public static final double kVangular = 3.5;
        public static final double kAangular = 0.1;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static final double kMaxSpeed = 0.3; // In meters per second
        public static final double kMaxAcceleration = 0.5; // 3 In meters per second per second
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
        public static final double kPVel = 0.062231;
    }
    }

