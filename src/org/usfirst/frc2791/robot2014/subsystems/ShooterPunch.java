/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014.subsystems;


import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.Robot2014;

/**
 *
 * @author Maxwell
 */
public class ShooterPunch extends Team2791Subsystem {
    //robot parts
    private static SpeedController windingMotor;
    private static DoubleSolenoid releaseSolenoid;
    private static AnalogChannel potVoltageReading;
    private static DigitalInput topSensor;
    private static DigitalInput midSensor;
    private static DigitalInput botSensor;
    private static DigitalInput botSensorBackup;
    //state vars
    private static boolean fire = false;
    private static boolean enabled = false;
    public Timer shotTimer;
    //0 top, 1 mid, 2 bot
    private static short lastSensorHit;
    //the amount of time in seconds to wait before starting to pull back the 
    //shooter again after a shot
    public final double SHOT_TIME = 1.0;
    private static boolean pullingBack = false;
//    private static boolean LOWERING = true;
    
    public ShooterPunch() {
        windingMotor = new Victor(1, 5);
        releaseSolenoid = new DoubleSolenoid(3,4);
        setReleaseSolonoid(false);
        
        shotTimer = new Timer();
        shotTimer.start();
        
        topSensor = new DigitalInput(8);
        midSensor = new DigitalInput(7);
        botSensor = new DigitalInput(6);
        botSensorBackup = new DigitalInput(9);
        readSensors();
    }
    
    public void test(){
        if(Robot2014.operatorStick.getRawButton(8))
            windingMotor.set(-.5);
        else
            windingMotor.set(0);
        setReleaseSolonoid(Robot2014.operatorStick.getRawButton(9));
    }
    
    public void setEnabled(boolean enabledIn) { enabled = enabledIn; }
    public boolean getEnabled() { return enabled; }
    
    public void run() {
        //instead of using a state macheine I decided to use some logic instead
        //this measures the consition that the shooter punch is currently in
        //stuff like, are we ready to fire, how much time has passed since the last
        //fire, ect and choses an action based on that. Impliments a simple bang
        //bang controller for position
       readSensors();
       //first thing first, if the shooter is not loaded load it
       if(fire) { //if time to fire, well FIRE
           setReleaseSolonoid(true);
           shotTimer.reset();
           fire = false;
       }
       if(enabled) {
            if(!readyToFire()) {
                //check if we are in a cooldown period, en
//                if(shotTimer.get() < SHOT_TIME) {
                if(pullingBack || !topSensor.get()) { //if hit the top sensor or after hit and still pulling
                    pullingBack = true;
                    setReleaseSolonoid(false);
//                    windingMotor.set(-0.8);
                    if(lastSensorHit == 0) //top sensor
                        windingMotor.set(-1.0);
                    else if(lastSensorHit == 1) //mid sensor
                        windingMotor.set(-0.50); //was -.80 then -.50
                } else { //waiting for punch to hit top sensor
                    pullingBack = true;
                    //wait and do nothing
                    windingMotor.set(0);
                }
            } else { //shooter is ready to fire
                pullingBack = false;
                windingMotor.set(0); //turn off the winding motor
            }
       } else { //disabled
           windingMotor.set(0);
       }
    }
    
    private void readSensors() {
        if(!topSensor.get())
            lastSensorHit = 0;
        else if(!midSensor.get())
            lastSensorHit = 1;
        else if (!botSensor.get() || !botSensorBackup.get())
            lastSensorHit = 2;
    }
    
    private void setReleaseSolonoid(boolean release) {
        if(release)
            releaseSolenoid.set(DoubleSolenoid.Value.kForward);
        else 
            releaseSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public boolean readyToFire() {
        return !botSensor.get() || !botSensorBackup.get();
//        return !midSensor.get();
    }
    
    //this method fires if able and returns if it was able to fire or not
    public boolean fire() {
       fire = true;
//        fire = false;
        return true;
        //don't fire if you're not ready
        
//        if(readyToFire()) {
//            fire = true;
//            return true;
//        } else {
//            return false;
//        }
    }
    
    public void disable() {
        fire = false;
        setEnabled(false);
        shotTimer.reset();
        readSensors();
    }
    
    public String getDebugString() {
        //will replace this with an actual method when we know what the sensor is
        String dbgStr = "Ready: ";
        if(readyToFire())
            dbgStr += "Y ";
        else
            dbgStr += "N ";
        dbgStr += "LstSenHit: "+lastSensorHit;
        return dbgStr;
        
    }
    
    public void setMotor(double set) {windingMotor.set(set);}
    
    public void display(){
        SmartDashboard.putBoolean("Top Sensor", !topSensor.get());
        SmartDashboard.putBoolean("Mid Sensor", !midSensor.get());
        SmartDashboard.putBoolean("Bot Sensor (6)", !botSensor.get());
        SmartDashboard.putBoolean("Bot Sensor Backup (9)", !botSensorBackup.get());
    }
}