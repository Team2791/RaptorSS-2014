/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014;

import edu.wpi.first.wpilibj.templates.Robot2014;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2791.robot2014.utils.Latch;

/**
 *
 * @author Maxwell
 */
public class TeleopRunner {
    
    //asorted constants
    final static double DRIVER_STICK_DEAD_ZONE = 0.08;
    final static double OPEREATOR_STICK_DEAD_ZONE = 0.13; //0.15
    public boolean SAFETY_MODE = true, PREP_SHOT = true;
    public String DRIVE_TYPE = "Linear";
    
    //vars needed to keep track of input state
//    private static Latch clawOpenLatch = new Latch(false);
    private static Timer autoCloseTimer = new Timer();
    private static Timer waitOnClaw = new Timer();
    private static Timer overrideArmAngleSensorTimer = new Timer();
    private static boolean manualClawControl = true;
    private static boolean autoCatch = false;
    
    private static final boolean opereatorJoysitck = false;
    private static boolean bottomPreset, tenPointPreset, trussPreset, groundPreset, killPIDButton;
    private static double armManualMovement;
    private static boolean shootButton, intakeOut, intakeIn, manualClawOpen, fireOverride;
    
    public static boolean highgear_default = false;
    
    public TeleopRunner() {
        autoCloseTimer.start();
        overrideArmAngleSensorTimer.start();
        reset();
    }
    
    public void reset() {
        autoCloseTimer.reset();
        overrideArmAngleSensorTimer.reset();        
    }
    
    public void setSafety(boolean set) { SAFETY_MODE = set; }
    
    public void run(){
        Robot2014.compressor.start();  //gotta love that air
        //update the drive motors
        driveTeleop();
        //keep arm commented out until tested
        armTeleop();  //update arm 
        shooterClawTeleop();
        
//        if(Robot2014.operatorStick.getRawButton(11) && Robot2014.operatorStick.getRawButton(10))
//            SAFETY_MODE = false; // disable safety
    }
    
    private void driveTeleop() {
        
        double leftSpeed, rightSpeed;
        
        //Robot2014.driveTrain.setLeftRightSpeed(scaleJoysticks(Robot2014.joystickLeft.getY()), scaleJoysticks(joystickRight.getY()));  
//        //use this to scale driving values(NOT LINEAR)
//        
//        //tank drive
//        rightSpeed = -Robot2014.driverRightStick.getY();
//        leftSpeed = -Robot2014.driverLeftStick.getY();
                
        // arcade drive
        //2 joysticks
//        double throttle = Robot2014.driverRightStick.getY();
//        double turn = Robot2014.driverLeftStick.getX();
        //1 gamepad
//        double throttle = 1.0 * Robot2014.driverLeftStick.getRawAxis(2);
//        double turn = 1.0 * Robot2014.driverLeftStick.getRawAxis(4);
        //racecar drive
        double throttle =  Robot2014.driverLeftStick.getRawAxis(3);
        double turn =  Robot2014.driverLeftStick.getRawAxis(1);
        
        leftSpeed = rightSpeed = -raiseToPower(throttle, 2.0);
        leftSpeed += raiseToPower(turn, 2.0);
        rightSpeed -= raiseToPower(turn, 2.0);
        SmartDashboard.putNumber("Throttle", throttle);
        SmartDashboard.putNumber("Turn amount", turn);
        SmartDashboard.putNumber("DT Left speed", leftSpeed);
        SmartDashboard.putNumber("DT Right speed", rightSpeed);
        if(Robot2014.driverLeftStick.getRawButton(1) == highgear_default)  {
            leftSpeed *= 0.6;
            rightSpeed *= 0.6;
        }
        Robot2014.driveTrain.setLeftRightSpeed(leftSpeed, rightSpeed);
    }
    
    private double raiseToPower(double x, double y) {
        boolean negative = x < 0;
        x = MathUtils.pow(Math.abs(x), y);
        if(negative) x = -x;
        return x;
    }
    
    private void armTeleop() {
        //if a preset button was pressed then go to that preset, however if 
        //the stick is outside of a dead zone cancel the PID and listen to the
        //stick commands     
        robotArmSensorDisable();
        
        if(opereatorJoysitck) {
            armManualMovement = Robot2014.operatorStick.getY();
            bottomPreset = Robot2014.operatorStick.getRawButton(4);
            tenPointPreset = Robot2014.operatorStick.getRawButton(3);
            trussPreset = Robot2014.operatorStick.getRawButton(5);
            groundPreset = Robot2014.operatorStick.getRawButton(8);
            killPIDButton = Robot2014.operatorStick.getRawButton(9);
        } else {
            armManualMovement = -Robot2014.operatorStick.getRawAxis(5);
            bottomPreset = Robot2014.operatorStick.getRawButton(2);
            tenPointPreset = Robot2014.operatorStick.getRawButton(3);
            trussPreset = Robot2014.operatorStick.getRawButton(4);
            groundPreset = Robot2014.operatorStick.getRawButton(1);
            killPIDButton = Robot2014.operatorStick.getRawButton(8);
        }
        
        if(armManualMovement > OPEREATOR_STICK_DEAD_ZONE || 
                armManualMovement < -OPEREATOR_STICK_DEAD_ZONE) {
            //will transfer to the adjusted manual output once it has been tested
//            Robot2014.robotArm.setMotorOutputManual(Robot2014.operatorStick.getY());
            Robot2014.robotArm.setMotorOutputManualAdjusted(-armManualMovement);
//             Robot2014.robotArm.setMotorOutputManualAdjusted(Robot2014.opereatorStick.getY());
        } else if (!Robot2014.robotArm.getUsePID()) {
           Robot2014.robotArm.setMotorOutputManualAdjusted(0.00);
        }
        
        if(bottomPreset) Robot2014.robotArm.goToPreset(0);
        else if(tenPointPreset) Robot2014.robotArm.goToPreset(1);
        else if(trussPreset) Robot2014.robotArm.goToPreset(4);
        else if(groundPreset) Robot2014.robotArm.goToPreset(3);
        else if(killPIDButton) Robot2014.robotArm.setMotorOutputManualAdjusted(0.0);
        
//        else if(Robot2014.operatorStick.getTrigger()) PREP_SHOT = true;
//        //ect
//        //etc
        
        /*if(PREP_SHOT){
            Robot2014.robotArm.goToPreset(2);
            if(Robot2014.robotArm.nearSetpoint())
                
        }*/
        
    }
    
    private void shooterClawTeleop() {
        Robot2014.shooterPunch.setEnabled(true); //trun of winch, done broke
//        Robot2014.shooterPunch.setEnabled(Robot2014.operatorStick.getRawButton(10));
        
        //first check auto catch
        //was button 11, now 9 to test
        autoCatch = false; //Robot2014.operatorStick.getRawButton(9); //could be button 10 too
        if(opereatorJoysitck) {
            shootButton = (Robot2014.driverLeftStick.getRawButton(8) && Robot2014.operatorStick.getTrigger());
            intakeOut = Robot2014.operatorStick.getRawButton(6);
            intakeIn = Robot2014.operatorStick.getRawButton(7);
            
            manualClawOpen = Robot2014.operatorStick.getRawButton(2);
            fireOverride = Robot2014.operatorStick.getRawButton(11);
        } else {
            //shootButton = (Robot2014.driverLeftStick.getRawButton(8) || Robot2014.operatorStick.getRawButton(6));
            shootButton = ( Robot2014.operatorStick.getRawButton(6));
            intakeOut = Robot2014.operatorStick.getRawAxis(3) < -0.5;
            intakeIn = Robot2014.operatorStick.getRawAxis(3) > 0.5;
            manualClawOpen = Robot2014.operatorStick.getRawButton(5);
            fireOverride = Robot2014.operatorStick.getRawButton(9);
        }
                
        if(autoCatch) { //auto catch
            Robot2014.intakeClaw.autoCatch(); //do auto catch things
//            clawOpenLatch.setManual(Robot2014.intakeClaw.getClawOpenRaw()); //sync up the latch and auto catch
        } else {//normal opereation
            //claw open close somewhere else
            //normal rollers
            if(intakeOut) //out
                Robot2014.intakeClaw.setIntakeRollerOutput(-1);
            else if(intakeIn) { //in
                double baseSpeed = 0.7;
                Robot2014.intakeClaw.setIntakeRollerOutput(baseSpeed + Robot2014.driveTrain.getSpeed()/160.0 * (1.0-baseSpeed));
            } else
                Robot2014.intakeClaw.setIntakeRollerOutput(0);
        }
        
        //shooting + claw auto
        if(manualClawOpen) { //only use latch when hit button
            manualClawControl = true;
        }
        if(shootButton && !autoCatch) { //trigger shooting
            //
            if(Robot2014.intakeClaw.getClawOpen()) {
                Robot2014.shooterPunch.fire();
                autoCloseTimer.reset();
            } else {
                Robot2014.intakeClaw.setClawOpen(true);
            }
            Robot2014.intakeClaw.setClawOpen(true);
            manualClawControl = false;
            
        } else if(fireOverride) {//no matter what shooting
            Robot2014.shooterPunch.fireOverride();
        } else if(manualClawControl && !autoCatch) {
            Robot2014.intakeClaw.setClawOpen(manualClawOpen);
        } else { //control of the arm is by shooter delay
            if(autoCloseTimer.get() >= 0.5 && !autoCatch) {
                Robot2014.intakeClaw.setClawOpen(false);
            }
        }
        
    }
    
    private void robotArmSensorDisable() {
//        boolean overrideArmAngleSensor = Robot2014.operatorStick.getRawButton(9);
//        if(overrideArmAngleSensor) {
//            if(overrideArmAngleSensorTimer.get() > 1.0)
//                Robot2014.robotArm.setArmSensorBroken(true);
//        } else {
//           overrideArmAngleSensorTimer.reset();
//        }
    }
}
