package frc.robot;

import java.lang.Math;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Mech {
    
    public CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushed);
    public CANSparkMax elevator = new CANSparkMax(11, MotorType.kBrushless);
    public CANSparkMax angle = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax cowCatch = new CANSparkMax(8, MotorType.kBrushless);
    public PIDController ElPid = new PIDController(1.45, 0, 0.2);
    public PIDController wristPID = new PIDController(.40, 0, 0.0);
    public PIDController cowPID = new PIDController(.55, 0, 0.0);
    public double kEncElConstant = 2 * Math.PI / 23;
    public RelativeEncoder elEncoder = elevator.getEncoder();
    public RelativeEncoder cowEnc = cowCatch.getEncoder();
    public RelativeEncoder wristEnc = angle.getEncoder();
    public double EL_LIMIT = 0.6;
    public double EL_LIMIT_DIST = 4.0;
    public double WRIST_SPEED_LIMIT = .2;
    public double COW_SPEED_LIMIT = .35;
    public double WRIST_ERROR_MIN = 0.75;
    public double elEndpoint = 1.725;
    public double wristEnd = 32;
    // Control and Safety Parameters
        // Encoder Boundaries (Note: 32 counts is equal to 1 Revolution)
        // NOTE ! ! ! : Make sure cow catcher and angle are set to "default" positions.
    final int COWENC_MAX = 30; // Assuming Cow Catcher starts folded into robot.
    final int ANGLE_MAX = 27; // Assuming angle starts folded on top of robot.

    public Mech() {
        //output in feet
        elEncoder.setPositionConversionFactor( kEncElConstant / 12 );
    }

    public void Intake(boolean ward, boolean back){
        // triggers
        if (ward) {
            intake.set(.65);
        }
        else if(back){
            intake.set(-.65);
        }
        else{
            intake.set(0);
        }
    }

    public void wristPID(boolean butt) {
        double curpoint = wristEnc.getPosition();
        
        if (butt) {
            double output = wristPID.calculate(curpoint, wristEnd);
            // Check that error is significant enough to justify correction.
            if (Math.abs(output) > WRIST_SPEED_LIMIT) {
                output = WRIST_SPEED_LIMIT * Math.signum(output);
            }
            angle.set(output);
        }
        else if (!butt) {
            double output = wristPID.calculate(curpoint, 0.2);
            // Check that error is significant enough to justify correction.
            if (Math.abs(output) > WRIST_SPEED_LIMIT) {
                output = WRIST_SPEED_LIMIT * Math.signum(output);
            }
            angle.set(output);
        }
    }

    public void cowPID(boolean butt) {
        double curpoint = cowEnc.getPosition();
        
        if (butt) {
            double output = cowPID.calculate(curpoint, COWENC_MAX);
            // Check that error is significant enough to justify correction.
            if (Math.abs(output) > COW_SPEED_LIMIT) {
                output = COW_SPEED_LIMIT * Math.signum(output);
            }
            cowCatch.set(output);
        }
        else if (!butt) {
            double output = cowPID.calculate(curpoint, 2);
            // Check that error is significant enough to justify correction.
            if (Math.abs(output) > COW_SPEED_LIMIT) {
                output = COW_SPEED_LIMIT * Math.signum(output);
            }
            cowCatch.set(output);
        }
    }

    public void Elevator(boolean butt){
        double curpoint = elEncoder.getPosition();
        if (butt) {
            double output = ElPid.calculate(curpoint, -elEndpoint); //Encoder's backwards.
            // Below, I attempted to have the elevator slowly increase in speed as it
            // gets going, but this failed.
            // We want the elevator to slowly pick up speed when it moves rather than
            // jolting harshly when it first moves.
                // Tried to have it so the "curpoint" controls the speed limit
                // while it's below the actual "EL_LIMIT" because "curpoint" starts
                // at around zero than should've increased over time.
            double limit = Math.abs(curpoint) < EL_LIMIT ? Math.abs(curpoint) : EL_LIMIT;
            if (Math.abs(output) > limit) {
                output = EL_LIMIT * Math.signum(output);
            }
            elevator.set(output);
        }
        else if (!butt) {
            double output = ElPid.calculate(curpoint, 0);
            // Below, I attempted to have the elevator slowly increase in speed as it
            // gets going, but this failed.
            // We want the elevator to slowly pick up speed when it moves rather than
            // jolting harshly when it first moves.
                // Tried to do the same thing, but using the difference between "curpoint"
                // and "endpoint", since the difference should've started at around
                // zero then increased as the elevator got further away from the "endpoint".
            double limit = Math.abs(Math.abs(curpoint) - Math.abs(elEndpoint)) < EL_LIMIT ? Math.abs(curpoint) : EL_LIMIT;
            if (Math.abs(output) > limit) {
                output = EL_LIMIT * Math.signum(output);
            }
            elevator.set(output);
        }
    }
    
    public void cowCatch(boolean ward, boolean back) {
        if (ward && cowEnc.getPosition() < COWENC_MAX) {
            cowCatch.set(0.4);
        }
        else if (back && cowEnc.getPosition() > 0) {
            cowCatch.set(-0.4);
        }
        else { 
            cowCatch.set(0.0);
        }
    }

    public void setIntake() {
        intake.set(.50);
    }

}