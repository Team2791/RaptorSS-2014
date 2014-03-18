/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014.subsystems;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2791.robot2014.BasicPID;
import org.usfirst.frc2791.robot2014.FloppityPID;
import edu.wpi.first.wpilibj.templates.Robot2014;
import java.util.Vector;

/**
 * For now I'm going to write this class like the drive train class because it is not
 * self contained like the shooter wheels. To make it self contained would result in a
 * lot of unnecessary junk. I'm trying to get away from the command robot framework
 * because it's large and cumbersome however I like the concept and would like to retain
 * some elements. This is a hard case where compromise comes in because it's an application.
 * that normally would be self containedRobot2014 but in this case is not.
 * @author Maxwell
 */
public class RobotArm extends Team2791Subsystem {
    public static SpeedController armMotor;
    //FOR NOW ASSUME ENCODER THIS LIKELY WILL CHANGE TO AN ANALOG ABSOLUTE SENSOR!!!!
    public static AnalogChannel absEncoder;
    //pratice bot angle offset 107.0
    //flight bot offset of 39.0 (should put these as prefs)
    //reading = sensorRaw - offset
    static double MIN_ANGLE = Robot2014.getDoubleAutoPut("Arm-Min_Angle",3.0);
    static double MAX_ANGLE = Robot2014.getDoubleAutoPut("Arm-Max_Angle",110.0);
    static double ANGLE_OFFSET = Robot2014.getDoubleAutoPut("Arm-Agnle_Offset",107.0);
    //offset is subtracted
    
//    public static BasicPID armPID;
    public static FloppityPID armPID;
    
    //these are the shooter PID constants
    //this is the tollerence that the wheel speed has to be within to be considered "good"
    private static double PID_P = 0.075, PID_I = 0.01, PID_D = 0.007, PID_DEADZONE = 3; //plus or minus
    private static double PID_DEADTIME = 0.0;
//    PID_P = 0.15, PID_I = 0.0ejeen  5, PID_D = 0.005;
    
    //this var helps the manual control code play nice with the PID code
    private static boolean usePID = false;
    
    //this is an array of presets
    //the presets are as follows: AUTON_SHOT, LOADING, TELEOP_BACK_SHOT
    //65.5 old angle, 75.0 crazy pratice field angle
    private static final double[] PRESET_VALUES = {22.5, 67.0, 90.0, 7.0};
    public boolean nearShooter = false;
    //debug stuff
    private static double lastAngle = 0;
    private static int sensorMessedUpCount = 0;
    private static final boolean angleSensorBrokenHard = true;
    private static boolean angleSensorBrokenSoft = false;
    
    
    public RobotArm() {
     //init the motors
        armMotor = new Victor(1, 3); //motor 7
        //look into putting this on the dashboard
        
        
        absEncoder = new AnalogChannel(1);
        
        PID_P = Robot2014.getDoubleAutoPut("Arm-PID_P",0.0);
        PID_I = Robot2014.getDoubleAutoPut("Arm-PID_I",0.0);
        PID_D = Robot2014.getDoubleAutoPut("Arm-PID_D",0.0);
        PID_DEADZONE = Robot2014.getDoubleAutoPut("Arm-PID_DEADZONE",0.0);
        PID_DEADTIME = Robot2014.getDoubleAutoPut("Arm-PID_DEADTIME",0.0);
        
        
        //init the PID
//        armPID = new BasicPID(PID_P, PID_I, PID_D);
        armPID = new FloppityPID(PID_P, PID_I, PID_D, PID_DEADZONE, PID_DEADTIME);
        armPID.setMaxOutput(1.0);
        armPID.setMinOutput(-1.0);
        
    }
    
    public void run(){
//        if(Robot2014.getBoolAutoPut("Arm-Sensor broken",false)) {
        if(angleSensorBrokenHard || angleSensorBrokenSoft) {
            usePID = false;
        } else {
            if(usePID) {
                double armAngle = getArmAngle();
                double PID_output = armPID.updateAndGetOutput(armAngle);
                if(armAngle > 84.0) {
                    if(PID_output < -0.2)
                        PID_output = -0.2;
                    else if(PID_output > 0.2)
                        PID_output = 0.2;
                }
                setMotorOutputAdjusted(PID_output);
            } else {
                //do nothing the output has already been set without the PID
                armPID.reset();
            }
        }
    }
    /**
     * This sets the arm motor output without any adjustment
     * @param output 
     */
    public void setMotorOutputManual(double output) {
        usePID = false;
//        System.out.println("arm set motor output manual called: "+output);
        setMotorOutput(output);
    }
    
    /**
     * this sets the arm motor output but it adjusts for the weight of the arm
     * so that the output is smoother
     * @param output 
     */
    public void setMotorOutputManualAdjusted(double output) {
        usePID = false;
        if(angleSensorBrokenHard || angleSensorBrokenSoft) {
            setMotorOutput(output);
            return;
        }
        
        double armAngle = getArmAngle();
        setMotorOutput(output - getFeedForward(armAngle));
//        System.out.println("arm set motor output manual adj called: "+(output - getFeedForward(armAngle)));
    }
    
    public void setMotorOutputAdjusted(double output) {
        if(angleSensorBrokenHard || angleSensorBrokenSoft) {
            setMotorOutput(output);
            return;
        }
        double armAngle = getArmAngle();
        SmartDashboard.putNumber("Arm output raw",output);
        SmartDashboard.putNumber("Arm FF output",-getFeedForward(armAngle));
        SmartDashboard.putNumber("Arm total output",output-getFeedForward(armAngle));
        setMotorOutput(output - getFeedForward(armAngle));
    }
    
    private void setMotorOutput(double output) {
        //motor spins other direction
        output = -output;
//        if(Robot2014.getBoolAutoPut("Arm-Sensor broken",false)) {
        
        //saturate output
        if(output > 1.0) //this should be up
            output = 1.0;
        else if(output < -0.3) //this should be down
            output = -0.3;
        
        //if we can read the angle sensor use it
        if(!(angleSensorBrokenHard || angleSensorBrokenSoft)) {
            if(armHitLowerLimit()) {
                if(output < 0)
                    output = 0;
            } else if(armHitUpperLimit()) {
                if(output > 0)
                    output = 0;
            }
        }
        //convert output to target voltage based on 12.5v battery, this makes it behave nicer
        output = output * 12.5;
        //calculate % of battery voltage the target output voltage is
        output = output/Robot2014.station.getBatteryVoltage();
        
        SmartDashboard.putNumber("Arm fianl output",output);
        armMotor.set(output);
    }
    
    public void goToPreset(int presetIndex) {
        //first make sure the preset exists
        if(presetIndex <0 || presetIndex >= PRESET_VALUES.length)
            return;
        usePID = true;
        armPID.setSetPoint(PRESET_VALUES[presetIndex]);
        armPID.reset();
        
    }
    
    public void setCustonSetpoint(double angle) {
        usePID = true;
        armPID.setSetPoint(angle);
        armPID.reset(); 
    }
    
    public void setArmSensorBroken(boolean broken) {
        angleSensorBrokenSoft = broken;
    }
    
    public void disable(){
        armMotor.set(0.0);
        
        MIN_ANGLE = Robot2014.getDoubleAutoPut("Arm-Min_Angle",3.0);
        MAX_ANGLE = Robot2014.getDoubleAutoPut("Arm-Max_Angle",110.0);
        ANGLE_OFFSET = Robot2014.getDoubleAutoPut("Arm-Agnle_Offset",107.0);
        
        PID_P = Robot2014.prefs.getDouble("Arm-PID_P",0.0);
        PID_I = Robot2014.prefs.getDouble("Arm-PID_I",0.0);
        PID_D = Robot2014.prefs.getDouble("Arm-PID_D",0.0);
        PID_DEADZONE = Robot2014.prefs.getDouble("Arm-PID_DEADZONE",0.0);
        PID_DEADTIME = Robot2014.getDoubleAutoPut("Arm-PID_DEADTIME",0.0);
        
//        armPID = new BasicPID(PID_P, PID_I, PID_D);
        armPID = new FloppityPID(PID_P, PID_I, PID_D, PID_DEADZONE, PID_DEADTIME);
        armPID.setMaxOutput(1.0);
        armPID.setMinOutput(-1.0);
        armPID.reset();
        usePID = false;
        
        angleSensorBrokenSoft = false;
    }
    
   private boolean armHitLowerLimit() { return getArmAngle()< MIN_ANGLE; }
   private boolean armHitUpperLimit() { return getArmAngle()> MAX_ANGLE; }
   
   public boolean nearShooterAngle(){
       if(getArmAngle()>=179 && getArmAngle()<=181)
           return true;
       else
           return false;
   }
    
   public double getPIDTarget() { return armPID.getSetPoint(); }
   public double getArmAngle() {
        //0v is not rotated at all, 5v is fully rotated
        //include an offset for weird mounting, also assume no wraparound
       //angle is a bit off 0 is right but 90 is 83
        double angle = ((5.0-absEncoder.getAverageVoltage()) * 360.0 / 5.0 - ANGLE_OFFSET );
        SmartDashboard.putNumber("Arm angle unfiltered", angle);
        if(angle > 150.0)
            setArmSensorBroken(true);
        return angle;
//        if(angle > 150.0) { //did have 250.0 but still got random high angle
//            SmartDashboard.putBoolean("Sensor Messed up",true);
//            sensorMessedUpCount++;
//            SmartDashboard.putNumber("Sensor Messed up count",sensorMessedUpCount);
//            return lastAngle;
//        } else {
//            SmartDashboard.putBoolean("Sensor Messed up",false);
//            lastAngle = angle;
//            return angle;
//        }
    }
    
    /**
     * This is where the magic happens, this code reads in the current arm location
     * and returns a feed forward required to keep it there
     * @return the motor output feed forward value
     */
    private double getFeedForward(double angle) {
        //as the angle gets larger the force of the motor required to keep it up
        //decreasses, so we use a cosign function
//        return Robot2014.dash.getNumber("RobotArm-FF_Const",0.0) * Math.cos(angle/180 * Math.PI);
        if(angle < 5) {
            return 0.1*Math.cos(angle/180*Math.PI);
        } else {//37 is gas shock 90
//            return 0.148*Math.cos(angle/180*Math.PI) + 0.05*Math.sin((angle+53)/180*Math.PI); // 0.09
            //angle offset used to be 36
            if(angle<35) {
                return 0.148*Math.cos(angle/180*Math.PI + 0.05);
            } else {
                return 0.148*Math.cos(angle/180*Math.PI + 0.05) + 0.03*Math.cos((angle-43)/180*Math.PI); // 0.148 orig, 0.165 pbot
            }
        }
            //0.148
        //in a different system this could be anythign from a switch statment used to
        //get the feed foward based on preset to a crazy polynomial or even a constant
    }
    
    public String getDebugString() {
        int angle = (int) getArmAngle();
        return "ArmAng:"+angle+" PID:"+armPID.getOutput();
    }
    
    //some code that will probably never be used
    public void setUsePID(boolean setUsePID) { usePID = setUsePID; }
    public boolean getUsePID() { return usePID; }
    
    
    public boolean nearSetpoint(){
        return (Math.abs(armPID.getError()) < 1.5);
    }
    
    public boolean nearSetpointMoving(){
        return (Math.abs(armPID.getError()) < 5.0);
    }
    
    public void display(){
        SmartDashboard.putNumber("Arm Angle",(int)getArmAngle());
        SmartDashboard.putNumber("Sensor Messed up count",sensorMessedUpCount);
        sensorMessedUpCount = 0;
        SmartDashboard.putNumber("PID Output",armPID.getOutput());
        SmartDashboard.putBoolean("Arm near setpoint",nearSetpoint());
        SmartDashboard.putBoolean("Arm near setpoint moving",nearSetpointMoving());
        SmartDashboard.putBoolean("Arm sensor broken", angleSensorBrokenHard || angleSensorBrokenSoft);
    }
}