package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import com.ctre.phoenixpro.hardware.Pigeon2;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import java.math.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; 
//import com.ctre.phoenix.sensors.Pigeon2;


public class Robot extends TimedRobot {
  private Double errorAdjust; 
  private Pigeon2 pig = new Pigeon2(7);
  private int setpoint;
  private double error;
  private Drivetrain drivetrain = new Drivetrain();
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  private static final String k_AutoEdge = "Default Edge Auton";
  private static final String k_AutoMiddle = "Middle Auton";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

 @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", k_AutoEdge);
    m_chooser.addOption("Middle Auto", k_AutoMiddle);
    SmartDashboard.putData("Auto Choices", m_chooser);

    m_led = new AddressableLED(9);

    m_ledBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_ledBuffer.getLength());

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
    }
 
    m_led.setData(m_ledBuffer);   
    m_led.start();
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
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto Selected: " + m_autoSelected);
    drivetrain.autonInnie();
    drivetrain.getTelem();
  }

  @Override
  public void autonomousPeriodic() {
    drivetrain.getTelem();
    switch (m_autoSelected) {
      case k_AutoMiddle:
        drivetrain.autonMiddle();
        break;
      case k_AutoEdge:
        drivetrain.auton();
        break;
      default:
        drivetrain.auton();
        break;
    }
    //drivetrain.auton();
    //drivetrain.autonMiddle();
  }

  @Override
  public void disabledInit() {
    drivetrain.getTelem();
  }
}