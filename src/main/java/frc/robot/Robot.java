// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  
  
  

  
  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushed);
 private CANSparkMax m_rightMotor = new CANSparkMax(3, MotorType.kBrushed);
 private DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
//  private CANSparkMax m_leftMotorRear = new CANSparkMax(1, MotorType.kBrushed);
//  private CANSparkMax m_rightMotorRear = new CANSparkMax(3, MotorType.kBrushed);
PS4Controller m_controller = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station


  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    // SendableRegistry.addChild(m_robotDrive, m_leftMotorRear);
    // SendableRegistry.addChild(m_robotDrive, m_rightMotorRear);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);


  }

  @Override
  public void teleopPeriodic() {
    // m_robotDrive.tankDrive(examplePS4.getRightY()/2, examplePS4.getLeftY()/2);
    m_robotDrive.tankDrive(-m_controller.getLeftY()/2, -m_controller.getRightY()/2);

    // m_robotDrive.tankDrive(-0.3, -0.3);
  }
}
