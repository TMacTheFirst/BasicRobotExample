// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  Joystick m_leftStick = new Joystick(0);
  Joystick m_rightStick = new Joystick(1);;
  XboxController controller = new XboxController(3);

  WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
  WPI_TalonSRX rightMotor = new WPI_TalonSRX(2);
  DifferentialDrive driveTrain = new DifferentialDrive(leftMotor, rightMotor);

  Solenoid solenoidExample = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  DoubleSolenoid doubleSolenoidExample = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);
  }

  @Override
  public void autonomousInit() {
    // Set up for autonomous
  }

  @Override
  public void autonomousPeriodic() {
    // Do autonomous
  }

  @Override
  public void teleopInit() {
    // Stop doing whatever autonomous was doing
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    /*
     * The output of GetRawButton is true/false depending on whether
     * the button is pressed; Set takes a boolean for whether
     * to use the default (false) channel or the other (true).
     */
    solenoidExample.set(m_leftStick.getRawButton(1));
   
    /*
     * In order to set the double solenoid, if just one button
     * is pressed, set the solenoid to correspond to that button.
     * If both are pressed, set the solenoid will be set to Forwards.
     */
    if (m_leftStick.getRawButton(2)) {
      doubleSolenoidExample.set(DoubleSolenoid.Value.kForward);
    } else if (m_leftStick.getRawButton(3)) {
      doubleSolenoidExample.set(DoubleSolenoid.Value.kReverse);
    }

    // XboxController example with toggle function
    if (controller.getYButtonPressed()) {
      solenoidExample.toggle();
      doubleSolenoidExample.toggle();
    }
    

    // Log encoder status to the dashboard.
    SmartDashboard.putNumber("Left Encoder Distance", leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Encoder Rate", leftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Encoder Distance", rightMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Rate", rightMotor.getSelectedSensorVelocity());
  }
}
