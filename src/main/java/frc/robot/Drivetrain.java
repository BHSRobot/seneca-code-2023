package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.networktables.*;

import java.io.IOException;
import java.math.*;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import frc.robot.Mech;


public class Drivetrain {
    private Mech mech = new Mech();
    private XboxController control1 = new XboxController(1);
    private XboxController control2 = new XboxController(2);
    private CANSparkMax dt_rl = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax dt_rf = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax dt_rf1 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax dt_ll = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax dt_lf1 = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax dt_lf = new CANSparkMax(6, MotorType.kBrushless);
    private MotorControllerGroup mc_left = new MotorControllerGroup(dt_ll, dt_lf, dt_lf1);
    private MotorControllerGroup mc_right = new MotorControllerGroup(dt_rl, dt_rf, dt_rf1);
    private DifferentialDrive dt_main = new DifferentialDrive(mc_left, mc_right);
    private RelativeEncoder dt_enc_1 = dt_ll.getEncoder();
    private RelativeEncoder dt_enc_2 = dt_rl.getEncoder(); //make more for safety precaution(elevator, cowcatcher, angle)
    private PIDController pid;
    private PIDController gyroPID = new PIDController(.3, 0, 0.15);;
    private final double kEncConstant = Math.PI * 6 / 8.46;
    private final double kP = 0.3; 
    private final double kI = 0.01; 
    private final double kD = 0.08; 
    private double distance;
    private int setpoint;
    private Pigeon2 pig = new Pigeon2(7);
    private boolean ele = false;
    private boolean wrist = false;
    private boolean goPID = false;
    private boolean gyroGo = false;
    private Timer timer = new Timer();
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private double x_off;
    private double h1 = 6.0;
    private double h2 = 63.0;
    private double k_limeAngle = 40.0;
    private double k_DT_LIMIT = .20;

    // Odometry Initialization
    DifferentialDriveOdometry odom;

    // Kinematics Initialization
        // Specify Robot's width.
    DifferentialDriveKinematics dt_kine = new DifferentialDriveKinematics(Units.inchesToMeters(27));

    public Drivetrain() {
        mc_left.setInverted(true);

        pid = new PIDController(kP, kI, kD);
        gyroPID = new PIDController(kP, 0, kD);

        dt_enc_1.setPositionConversionFactor(kEncConstant/12);
        dt_enc_2.setPositionConversionFactor(kEncConstant/12);

        // Initializaing Odometry for Differential Drive.
            // Get Rotation from IMU, and positions from left and right encoders.
        Rotation2d rot = new Rotation2d(pig.getYaw());
        double enc_L_dist = dt_enc_1.getPosition();
        double enc_R_dist = dt_enc_2.getPosition();
            // Create Odometry Object from recorded values.
        odom = new DifferentialDriveOdometry(rot, enc_L_dist, enc_R_dist);
        // NOTE ! ! ! : Put the fucntion "odom.update(new Rotation2d(pig.getYaw()), dt_enc_1.getPosition(), dt_enc_2.getPosition())"
        // into a periodic function so it is continuously updated.
    }

    public void drive() {
        if (control1.getAButtonPressed()) {
            gyroGo = !gyroGo;
        }
        if(control1.getPOV()==0){
            correctPos();
        }
        dt_main.tankDrive(control1.getLeftY()*.85, control1.getRightY()*.85);

        mech.Intake(control2.getBButton(), control2.getYButton());

        if(control2.getAButtonPressed()) {
            ele = !ele;
        }

        if (control2.getXButtonPressed()) {
            wrist = !wrist;
        }

        mech.wristPID(wrist);
        mech.Elevator(ele);
        gyroCorrect();

        //mech.inAngle(control2.getRightTriggerAxis() > 0, control2.getLeftTriggerAxis() > 0);
        
        //cow(KILLER)catcher
        mech.cowCatch(control1.getRightTriggerAxis() > 0, control1.getLeftTriggerAxis() > 0);
                
        /*if (control1.getBButton()) {
            pidCorrect();          
        }*/
        
    }

    public void correctPos() {
        dt_enc_1.setPosition(0.0);
        dt_enc_2.setPosition(0.0);

        while (dt_enc_1.getPosition() > -.5 && dt_enc_2.getPosition() >-.5) {
            dt_main.tankDrive(.3, .3);
        }
    }
    
    public void gyroCorrect() {   
        if (gyroGo) {
            if (Math.abs(pig.getPitch())> 1) {
                double output = gyroPID.calculate(pig.getPitch(), 1);
                if (Math.abs(output) > k_DT_LIMIT) {
                    output = k_DT_LIMIT * Math.signum(output);
                }
                dt_main.tankDrive(-output, -output);
            }
        }
    }

    public void pidCorrect() {
        goPID = !goPID;
        double y_off = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        if (goPID = true && ( x_off > 1.5 || x_off < -1.5) ) {
            double errorAdjust = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            errorAdjust *= 0.05;
            dt_main.tankDrive(-errorAdjust, errorAdjust);
        }
        if (goPID = true && ( x_off < 1.5 || x_off > -1.5 ) ) {
            distance = ( h2 - h1 ) / Math.tan( k_limeAngle + y_off );
            double output = pid.calculate((dt_enc_1.getPosition() * kEncConstant / 12.0), distance);
            if(Math.abs(output) > 1) {
                dt_main.tankDrive(-0.75, -0.75);
            }
            else {
                dt_main.tankDrive(-output*.75, -output*.75);
            }
        }
    }
    

    public void autonInnie(){
        gyro.reset();
        timer.reset();
        gyro.calibrate();
        timer.reset();
    }
    public void auton() {
        timer.start();
        SmartDashboard.putNumber("Timer", timer.get());
        mech.elevator.set(0.0);
        mech.angle.set(0.0);
        mech.intake.set(0.0);
        dt_main.tankDrive(0.0,0.0);
        double time = 15.0 - timer.get();

        SmartDashboard.putNumber("Time Test", time);

        if (time > 13.0 && mech.elEncoder.getPosition() > -1.7) {
            mech.elevator.set(-.40);
        }
       else if (time > 8.0 && mech.wristEnc.getPosition() < 25) {
            mech.angle.set(.15);
        }
        else if (time > 7.5)  {
            mech.intake.set(-.3);
           
        }
        else if (time>4.5 && mech.wristEnc.getPosition() >10){
         mech.angle.set(-.20);
        }
        else if(time>3.0){
            dt_main.tankDrive(.55, .55);
        }
    }

    public void getTelem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry x_off = table.getEntry("tx");
        NetworkTableEntry y_off = table.getEntry("ty");
        double tx = x_off.getDouble(0.0);
        double ty = y_off.getDouble(0.0);
        SmartDashboard.putNumber("X-Off", tx);
        SmartDashboard.putNumber("Ybb-Off", ty);
        SmartDashboard.putNumber("First Encoder Value", dt_enc_1.getPosition());
        SmartDashboard.putNumber("Robo X", odom.getPoseMeters().getX());
        SmartDashboard.putNumber("Robo Y", odom.getPoseMeters().getY());
        SmartDashboard.putNumber("Yaw", pig.getYaw());
        SmartDashboard.putNumber("Pitch", pig.getPitch());
        SmartDashboard.putNumber("Roll", pig.getRoll());
        SmartDashboard.putNumber("Cow Encoder", mech.cowEnc.getPosition());
        SmartDashboard.putNumber("Elevator Encoder", mech.elEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Encoder", mech.wristEnc.getPosition());
    }

    public void updateSensors(){
        odom.update(new Rotation2d(pig.getYaw()), dt_enc_1.getPosition(), dt_enc_2.getPosition());
    }
}