// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
// import pabeles.concurrency.IntRangeObjectConsumer;
import frc.robot.Robot.SwerveSubsystem;
// import java.security.PublicKey;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.MotorSafety;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Robot extends TimedRobot {
  private XboxController controller;
  private WPI_TalonSRX motor;
  private DigitalInput sensor;
  private Timer intakeTimer;
  private SwerveSubsystem swerve = new SwerveSubsystem();

 //Swarve drive ------------------------------------

//swarve drive ----------------------------------------
public class SwerveSubsystem extends SubsystemBase {

  private SwerveModule frontLeft = new SwerveModule(1, 2);   // Drive=1, Steer=2
  private SwerveModule frontRight = new SwerveModule(3, 4);  // Drive=3, Steer=4
  private SwerveModule backLeft = new SwerveModule(5, 6);    // Drive=5, Steer=6
  private SwerveModule backRight = new SwerveModule(7, 8);   // Drive=7, Steer=8

  public SwerveSubsystem() {
    System.out.println("Swerve subsystem created with 4 modules!");
  }

  public void testAllWheels() {
    frontLeft.setDriveSpeed(0.3);
    frontRight.setDriveSpeed(0.3);
    backLeft.setDriveSpeed(0.3);
    backRight.SetDriveSpeed(0.3);
}
public void stopAll() {
  frontLeft.stop();
  frontRight.stop();
  backLeft.stop();
  backRight.stop();
}
}

public void simpleDrive(XboxController controller) {
  double forward = -controller.getLeftY();  // Forward/backward
  double turn = controller.getRightX();     // Left/right turn
  
  // Tank drive math
  double leftSpeed = forward + turn;
  double rightSpeed = forward - turn;
  
  // Left side modules
  frontLeft.setDriveSpeed(leftSpeed);
  backLeft.setDriveSpeed(leftSpeed);
  
  // Right side modules  
  frontRight.setDriveSpeed(rightSpeed);
  backRight.setDriveSpeed(rightSpeed);
}


public class SwerveModule {
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  
  public SwerveModule(int driveID, int turnID) {
      driveMotor = new TalonFX(driveID);
      turnMotor = new TalonFX(turnID);

      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      turnMotor.setNeutralMode(NeutralModeValue.Brake);
  
  }
  public void SetDriveSpeed() {
    driveMotor.set(speed);
  }
  public void SetTurnSpeed() {
    turnMotor.set(speed);
  }
  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
  
  // Methods to control this wheel
}

  public Robot() {

  }
  @Override
  public void robotInit() {
    //ftc intake motor workshop
    motor = new WPI_TalonSRX(1); // change id later to the one you need, BS: chnage the WPI_TalonSRX to the kind of motor that ur using in here were using Falcon 500
    controller = new XboxController(0);
    sensor = new DigitalInput(0); // DIO port 0
    intakeTimer = new Timer();
    System.out.println("Motor robot initialized! along with the sensor");
    //swarve drivertrain

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
    swerve.simpleDrive(controller);
  }
  //This function is called once when the robot is disabled. 
  @Override
  public void disabledInit() {
    motor.stopMotor();
    swerve.stopAll();
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
