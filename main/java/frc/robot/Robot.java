package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenixpro.hardware.Pigeon2;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import java.math.*;
//import com.ctre.phoenix.sensors.Pigeon2;


public class Robot extends TimedRobot {
  private Double errorAdjust; 
  private Pigeon2 pig = new Pigeon2(7);
  private int setpoint;
  private double error;
  private Drivetrain drivetrain = new Drivetrain();

 @Override
  public void robotInit() {
    drivetrain.getTelem();
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.getTelem();
    drivetrain.drive();
    drivetrain.updateSensors();
  }
  
  @Override
  public void autonomousInit() {
    drivetrain.autonInnie();
    drivetrain.getTelem();
  }

  @Override
  public void autonomousPeriodic() {
    drivetrain.getTelem();
    drivetrain.auton();
  }

  @Override
  public void disabledInit() {
    drivetrain.getTelem();
  }
}