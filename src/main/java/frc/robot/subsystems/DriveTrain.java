// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// for drive module
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

// for swerve module
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.math.util.Units;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;


public class SwerveModule {

  // elements needed for the module
  private CANSparkMax driveMotor;
  private CANSparkMax steerMotor;
  private CANcoder    absoluteEncoder;
  private SparkMaxPIDController drivingPIDController;
  private SparkMaxPIDController turningPIDController;
  private RelativeEncoder driveEncoder;
  private RelativeEncoder steerEncoder;

  // initiation
  // sorry for the sparse commenting, but a lot of it is relatively straight forward logic-wise
  public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID) {
    // setup connections to robot, a big part of swerve is grabbing and applying very precise info
    driveMotor = new CANSparkMax(driveMotorCANID);
    steerMotor = new CANSparkMax(steerMotorCANID);
    absoluteEncoder = new CANcoder(cancoderCANID);

    // seperate pids/encoders for drive/turn
    drivingPIDController = driveMotor.getPIDController();
    turningPIDController = steerMotor.getPIDController();
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();

    // set everything to factory default at start
    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();
    absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
    // any additional configuration would go here

    // configurate the CANcoder
    CANcoderConfigurator cfg = encoder.getConfigurator();
    cfg.apply(new CANcoderConfiguration());
    MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
    cfg.refresh(magnetSensorConfiguration);
    cfg.apply(magnetSensorConfiguration
              .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
              .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    // change if you want to inverse the steering motor, some motors are
    // reversed so they only work correctly if inverse is enabled
    steerMotor.setInverted(false);
    turningPIDController.setFeedbackDevice(steerEncoder);

    // conversion factors for the turning encoder needed in radians for WPILib's swerve APIs
    // !!! i'm ripping this from the docs, idk if ModuleConstants is defined elsewhere if this doesn't work
    steerEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    steerEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // allows PID to wrap around 360 to 0 instead of going the long way during calculations
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(0);
    turningPIDController.setPositionPIDWrappingMaxInput(90);

    // idk what pid gains are
    turningPIDController.setP(ModuleConstants.kTurningP);
    turningPIDController.setI(ModuleConstants.kTurningI);
    turningPIDController.setD(ModuleConstants.kTurningD);
    turningPIDController.setFF(ModuleConstants.kTurningFF);

    // drive config

    // inverting is same as earlier, might be necessary
    driveMotor.setInverted(false);
    drivingPIDController.setFeedbackDevice(driveEncoder);

    // converting from rotations and RPM to meters and mps to using with WPILib
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);        
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // pid gains
    drivingPIDController.setP(ModuleConstants.kDrivingP);
    drivingPIDController.setI(ModuleConstants.kDrivingI);
    drivingPIDController.setD(ModuleConstants.kDrivingD);
    drivingPIDController.setFF(ModuleConstants.kDrivingFF);

    // save SPARK MAX config in case of browning out
    driveMotor.burnFlash();
    steerMotor.burnFlash();

    // finalizing
    driveEncoder.setPosition(0);
    steerEncoder.setPosition(encoder.getAbsolutePosition().refresh().getValue() * 360);
  }

  // following is grabbed from yagsl docs, basic commands
  /**
  Get the distance in meters.
  */
  public double getDistance()
  {
      return driveEncoder.getPosition();
  }
  
  /**
  Get the angle.
  */
  public Rotation2d getAngle()
  {
        return Rotation2d.fromDegrees(steerEncoder.getPosition());
  }
  
  /**
  Set the swerve module state.
  @param state The swerve module state to set.
  */
  public void setState(SwerveModuleState state)
  {
        turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
  }
}


//Link to libary for navx board or in other words the thing we use for a gyroscope thats in the center of the robot 
//Link -> https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
public class DriveTrain extends SubsystemBase {
  
  //Used from navx board (gyroscope that is currently on the center of the robot)
  //Roll, Pitch, Yaw
  private final double xAxis, yAxis, zAxis; 
  private AHRS gyro;

  // Essential attributes for swerve
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry   odometry;
  // !!! The following objects are psuedo, for this to work they need to exist             
  SwerveModule[] swerveModules;


  public DriveTrain() {
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
    gyro = new AHRS();
    gyro.enableLogging(true);

    // Initilizes with the current angle, and all the positional values are assumed to start here (x=0, r=0, heading=0)
    odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // Grab the angle as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
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
      odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),  getCurrentSwerveModulePositions());
  }
}

