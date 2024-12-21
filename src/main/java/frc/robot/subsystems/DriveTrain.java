// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {
  
  
  // Essential attributes for swerve
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry   odometry;
  // !!! The following objects are psuedo, for this to work they need to exist
  Gyroscope             gyro;
  SwerveModule[]        swerveModules;


  public DataTrain() {
    // !!! Swerve modules need to be setup
    swerveModules = new SwerveModule[4];

    // SwerveDriveKinematics object initilized here, allows the translating of chassis vars to module vars (x/y becomes speed/angle)
    // Holds locational information on the robot's module locations
    // todo: Add in the actual locational data
    kinematics = new SwerveDriveKinematics(
      // Object only works with meters
      // Swerve modules status output order depends off of following object order
      // Translation2d(x, y): x is vertical (+ is front, - is back) / y is horizontal (+ is left, - is right)
      new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
      new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
      new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
      new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
    );

    // Gyroscope initilization
    gyro = new Gyroscope();

    // Initilizes with the current angle, and all the positional values are assumed to start here (x=0, r=0, heading=0)
    odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getAngle(), // Grab the angle as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition},
            // Front-Left, Front-Right, Back-Left, Back-Right -> Keep in same order as kinematic setup
            new Pose2d(0,0,new Rotation2d()) // x=0, y=0, heading=0
        );
  }

  // !!! Following boiler plate is grabbed from https://docs.yagsl.com/fundamentals/swerve-drive
  // Don't edit the following unless it doesn't work, very important for robot
  
  // Fetch the current swerve module positions.
  public SwerveModulePosition[] getCurrentSwerveModulePositions()
  {
      return new SwerveModulePosition[]{
          new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
          new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
          new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
          new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
      };
  }
  
  @Override
  public void periodic()
  {
      // Update the odometry every run.
      odometry.update(gyro.getAngle(),  getCurrentSwerveModulePositions());
  }
}

