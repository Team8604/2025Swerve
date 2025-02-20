// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  Joystick logitechController = new Joystick(0);;
  SparkMax leftMotor = new SparkMax(10, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(20, MotorType.kBrushless);
  SparkMax intakeMotor = new SparkMax(30, MotorType.kBrushless);

  // For stopping intake motor when coral is intaked
  double intakeSpeed = 0, temp;
  boolean isCoral = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    leftMotor.set(Math.pow(logitechController.getRawAxis(1), 3));
    rightMotor.set(MathUtil.clamp(Math.pow(logitechController.getRawAxis(5), 3), -0.2, 0.2));

    // stopper for when coral inside of intake below
    temp = MathUtil.clamp(Math.pow(logitechController.getRawAxis(3) - logitechController.getRawAxis(2), 3), -0.3, 0.3);

    if (temp < 0) {
      // would mean intake backing out
      isCoral = false;
    } else if (isCoral || intakeMotor.getOutputCurrent() > 20) {
      // would mean already tripped or should based on current
      temp = 0;
      isCoral = true;
    }

    // sets speed to intake motor
    intakeSpeed = temp;
    intakeMotor.set(intakeSpeed);

    // display numbers
    SmartDashboard.putNumber("Bus voltage", rightMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake voltage", intakeMotor.get());
    SmartDashboard.putNumber("Intake speed", intakeSpeed);
    SmartDashboard.putNumber("Intake amps", intakeMotor.getOutputCurrent());
    SmartDashboard.putBoolean("isCoral", isCoral);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
