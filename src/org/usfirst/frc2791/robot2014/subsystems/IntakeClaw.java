/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014.subsystems;


import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.Robot2014;

/**
 *
 * @author Maxwell
 */
public class IntakeClaw extends Team2791Subsystem {
    //robot parts
    private static SpeedController intakeRollerVictor;
    private static DoubleSolenoid clawOpenSolenoid;
//    private static Solenoid clawOpenSolenoid;
    public static AnalogChannel ballRangeSensor;
    //varaibles to keep track fo state
    public static Timer clawMovementTimer;
    private static boolean clawOpen = false;
    private static double intakeRollerOutput = 0;
    private static boolean autoCatchFirstCall = true;
    private static boolean caught = false;
    private static boolean prevClawOpen = clawOpen;
    //cosntants
    //the time it takes to claw to open or close in seconds, use this for 
    //keeping track of if the claw is open or not
    private static final double CLAW_OPEN_CLOSE_DELAY = 0.000;
    
    public IntakeClaw() {
        intakeRollerVictor = new Victor(1, 4);
	LiveWindow.addActuator("Intake claw", "Intake roller victor", (Victor) intakeRollerVictor);
        clawOpenSolenoid = new DoubleSolenoid(1,2);
//        clawOpenSolenoid = new Solenoid(1);
        setClawSolenoidOpen(false);
        
        ballRangeSensor = new AnalogChannel(3);
        clawMovementTimer = new Timer();
        clawMovementTimer.start();
        clawMovementTimer.reset();
    }
    
    public void disable() {
        intakeRollerOutput = 0;
        run();
    }

    public void run() {
        setClawSolenoidOpen(clawOpen);
        //normalize intake roller output for voltage
        double output = (intakeRollerOutput*12.5)/Robot2014.station.getBatteryVoltage();
        intakeRollerVictor.set(output);
    }
    
    public void setIntakeRollerOutput(double output) { intakeRollerOutput = output; }
    public double getIntakeRollerOutput() { return intakeRollerOutput; }
    
    public void setClawOpen(boolean clawOpenSet) {
        autoCatchFirstCall = true;
        prevClawOpen = clawOpen;
        clawOpen = clawOpenSet;
        //noramlly we only set the solenoid in run, because that's where all the
        //robot actions get carried out BUT I'm making an exception so the timer is
        //accurate
        setClawSolenoidOpen(clawOpen);
        if(prevClawOpen != clawOpen)
            clawMovementTimer.reset();
    }
    
    private void setClawSolenoidOpen(boolean open) {
        //double solenoid code
        if(open)
            clawOpenSolenoid.set(DoubleSolenoid.Value.kForward);
        else
            clawOpenSolenoid.set(DoubleSolenoid.Value.kReverse);
        //single solenoid code
//            clawOpenSolenoid.set(open);\
    }
    
    //this method returns if the claw is open or not
    public boolean getClawOpen() {
        //check how much time has passed since the claw was told to move
        //if enough time has passed assume the claw has moved, otherwise assume
        //is hasn't
        return clawOpen;
//        if (clawMovementTimer.get() >= CLAW_OPEN_CLOSE_DELAY)
//            return clawOpen;
//        else
//            return !clawOpen;
    }
    
    public boolean getClawOpenRaw() { return clawOpen; }
    
    public double getBallRange() {
        //NO IDEA how the sensor reads for now just wing it
        if(ballRangeSensor.getAverageVoltage() < 0.1) //this is the min voltage
            return 2791; //as good an arbitrary large number as any
        //0.02 but sometimes 0.04 is base voltage ( i think), at 9.5 voltage is 0.2, 0 is 1.95
        return  (1.95- ballRangeSensor.getAverageVoltage())*(9.5/(1.95-0.2)) ;
//        return 0;
    }
    
    // open the claw if first call and then when the sensor reads a value in a range close the shooter
    public void autoCatch() {
        setIntakeRollerOutput(1); //roller time
        //check if ball is already there, or if 
        if(autoCatchFirstCall) {
            setClawOpen(true);
            caught = false;
        }
        
        //once we've caught it stop reading the sensor because the ball is inside it's min range
        if(!caught) { 
            if(getBallRange() < 9.1) { //was 6.6
                setClawOpen(false);
                caught = true;
            }
        } else { //if caught hold closed
            setClawOpen(false);
        }
    }
    
    public String getDebugString() {
        String debugString;
        if(getClawOpen())
            debugString = "OPEN  ";
        else
            debugString = "CLOSE ";
        debugString += " RollerOutput";
        return debugString;
    }
    
    public void display(){
        SmartDashboard.putNumber("claw open timer", clawMovementTimer.get());
//        SmartDashboard.putNumber("Intake output", intakeRollerOutput);
        SmartDashboard.putNumber("Ball Range",getBallRange());
        SmartDashboard.putNumber("ballRangeSensor Raw", ballRangeSensor.getAverageVoltage());
        SmartDashboard.putBoolean("Claw Open",getClawOpen());
    }
}