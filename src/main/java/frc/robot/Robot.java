// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
// import pabeles.concurrency.IntRangeObjectConsumer;

// import java.security.PublicKey;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.MotorSafety;


public class Robot extends TimedRobot {
  private XboxController controller;
  private WPI_TalonSRX motor;
  private DigitalInput sensor;
  private Timer intakeTimer;

  public Robot() {

  }
  @Override
  public void robotInit() {
    motor = new WPI_TalonSRX(1); // change id later to the one you need
    controller = new XboxController(0);
    sensor = new DigitalInput(0); // DIO port 0
    intakeTimer = new Timer();
    System.out.println("Motor robot initialized! along with the sensor");
  }

// this function is getting called every 20ms 
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  //This function is called once when teleop is enabled. 
  @Override
  public void teleopInit() {}

  // This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() {
    motor.configPeakCurrentLimit(30);
    Boolean intakemotor = controller.getAButton();
    Boolean outtakeIntake = controller.getBButton();
    if(outtakeIntake) {
      intakeTimer.start();
      motor.set(-0.3);
    }
    else if(!outtakeIntake) {
      motor.set(0);
    }

    if(intakemotor) {
      intakeTimer.start();
      motor.set(0.3);
    }
    else if(!intakemotor) {
      motor.set(0);
    }
    boolean sensorTriggered = !sensor.get();

    if(sensorTriggered) {
        motor.set(0);
        System.out.println("Sensor is trigger");
    }
    if(intakeTimer.get() > 5) {
      motor.set(0);
      System.out.println("ran out of time motor is coming to a stop");
    }
    else if(intakeTimer.get() < 5) {
      motor.set(0.5);
    }
    else{
      motor.set(0);
      intakeTimer.stop();
      intakeTimer.reset();
    }
  }
  //This function is called once when the robot is disabled. 
  @Override
  public void disabledInit() {
    motor.stopMotor();
  }

  // This function is called periodically when disabled. 
  @Override
  public void disabledPeriodic() {}

  // This function is called once when test mode is enabled. 
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
// hellooooo