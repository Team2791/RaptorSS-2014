/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014;

import edu.wpi.first.wpilibj.templates.Robot2014;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Vector;

/**
 *
 * @author Maxwell
 */
public class AutonRunner {
    private static int autonState, disksShot;
    private static Timer timer;
    private static BasicPID driveAnglePID, driveDistancePID;
    private static double PID_P, PID_I, PID_D;
    
    private static boolean twoBall = false;
    
    public AutonRunner() {
        System.out.println("AutonRunner init running");
        autonState = 1;
        timer = new Timer();
        timer.start();
        Vector keys = Robot2014.prefs.getKeys();
        PID_P = Robot2014.getDoubleAutoPut("DriveAngle-PID_P",0.0);
        PID_I = Robot2014.getDoubleAutoPut("DriveAngle-PID_I",0.0);
        PID_D = Robot2014.getDoubleAutoPut("DriveAngle-PID_D",0.0);
        driveAnglePID = new BasicPID(PID_P, PID_I, PID_D);
        driveAnglePID.setSetPoint(0.0);
        driveAnglePID.setMaxOutput(0.5);
        driveAnglePID.setMinOutput(-0.5);
        driveDistancePID = new BasicPID(Robot2014.getDoubleAutoPut("DriveDistance-PID_P",1.0/8.0), 
                0.0, Robot2014.getDoubleAutoPut("DriveDistance-PID_D",0.001));
        driveDistancePID.setMaxOutput(0.75);
        driveDistancePID.setMinOutput(-0.75);

        SmartDashboard.putNumber("Auton State", autonState);
        reset();
    }
    
    public void reset() {
        autonState = 1;
        timer.reset();
        PID_P = Robot2014.getDoubleAutoPut("DriveAngle-PID_P",0.0);
        PID_I = Robot2014.getDoubleAutoPut("DriveAngle-PID_I",0.0);
        PID_D = Robot2014.getDoubleAutoPut("DriveAngle-PID_D",0.0);
        driveAnglePID = new BasicPID(PID_P, PID_I, PID_D);
        driveAnglePID.setSetPoint(0.0);
        
        driveAnglePID.setMaxOutput(0.5);
        driveAnglePID.setMinOutput(-0.5);
        driveAnglePID.reset();
        driveDistancePID = new BasicPID(Robot2014.getDoubleAutoPut("DriveDistance-PID_P",1.0/8.0), 
                0.0, Robot2014.getDoubleAutoPut("DriveDistance-PID_D",0.001));
        SmartDashboard.putNumber("Auton State", autonState);
        driveDistancePID.setMaxOutput(0.75);
        driveDistancePID.setMinOutput(-0.75);
    }
    
    public void run() {
//        Robot2014.compressor.start(); //air is nice, except we're off board
        
        //depending the state of the autonState var a different case is run
        //this is called a state machiene and great for doing things in secquence
        //in a loop. When each function has done everything it needs to it returns true
        //and the state is bumped so on the next itteratio of the loop the next state
        //will run. case 0 is a the default do nothing case
        SmartDashboard.putNumber("Auton State", autonState);
        // various autons
        oneBallShlub();
//        oneBallOnePoint();
//            oneToTwoBall();
//        oneBallMoving();
        
//        driveElims();
        
    }
    
    private boolean oneBallOnePoint() {
        switch(autonState){
            case 1: //init loop
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                Robot2014.driveTrain.resetSensors();
                Robot2014.driveTrain.freeRangeGyro();
                driveDistancePID.setMaxOutput(0.50);
                driveDistancePID.setMinOutput(-0.50);
                timer.reset();
                autonState++;
                break;  
                
            case 2: //wait for sensors to reset, not sure how much time they need, 0.2 seems to be working
                if(timer.get() > 0.2)
                    autonState++;
                break;
                
            case 3:
                Robot2014.shooterPunch.setEnabled(true);
                Robot2014.intakeClaw.setClawOpen(false);
                Robot2014.robotArm.goToPreset(0);
                if(driveToDistance(13*12+4)) {
                    timer.reset();
                    autonState++;
                }
                break;
            case 4:
                if(Robot2014.robotArm.nearSetpoint())
                    Robot2014.intakeClaw.setIntakeRollerOutput(-1.0);
                else
                    timer.reset();
                
                if(timer.get() > 1.0) {
                    timer.reset();
                    Robot2014.intakeClaw.setIntakeRollerOutput(0.0);
                    autonState = 0;
                }
                break;
                
            case 0: default:
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                return true;
        }
        
        return false;
    }
    
    private boolean oneBallShlub() {
        switch(autonState){
            case 1: //init loop
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                Robot2014.driveTrain.resetSensors();
                timer.reset();
                autonState++;
                break;  
                
            case 2: //wait for sensors to reset, not sure how much time they need, 0.2 seems to be working
                if(timer.get() > 0.2)
                    autonState++;
                break;
                
            case 3:
                //oen pt goal normal 13*12+4
                //added 12
                if(driveToDistance(6*12)) {
                    timer.reset();
                    autonState = 0;
                }
                break;
                
            case 4:
                timer.reset();
                Robot2014.driveTrain.setLeftRightSpeed(0.2, 0.2);
                break;
            
            case 0: default:
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                return true;
        }
        
        return false;
    }
    
    private boolean oneToTwoBall() {
        switch(autonState){
            //lower the robot arm until it hits the limit switch and zeros out
            case 1: //init loop
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                Robot2014.driveTrain.resetSensors();
                Robot2014.driveTrain.freeRangeGyro();
                driveDistancePID.setMaxOutput(0.60);
                driveDistancePID.setMinOutput(-0.60);
                timer.reset();
                autonState++;
                break;  
                
            case 2: //wait for sensors to reset, not sure how much time they need, 0.2 seems to be working
                if(timer.get() > 0.2)
                    autonState++;
                break;
                
            case 3: //drive to shooting location
                Robot2014.shooterPunch.setEnabled(true);
                Robot2014.intakeClaw.setClawOpen(false);
                Robot2014.intakeClaw.setIntakeRollerOutput(0.7);
                Robot2014.robotArm.goToPreset(1);
                //crazy pratice field addation +3+9+5+12
                if(driveToDistance(12.833333*12-2)) {
                    timer.reset();
                    autonState++;
                }
                break;
                
            case 4: //wait for the arm to be good then kill the PID and fire
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                //kill pid
                if(Robot2014.robotArm.nearSetpoint()) {
                    Robot2014.robotArm.setMotorOutputManualAdjusted(0.0);
                    Robot2014.intakeClaw.setClawOpen(true);
                    Robot2014.shooterPunch.fire();
                    timer.reset();
                    autonState++;
                }
                    break;
                
              
            case 5: //wait for shot to finish, then get ready to catch
                if(timer.get() > .5) {
                    Robot2014.intakeClaw.setClawOpen(false);
                    Robot2014.driveTrain.setLeftRightSpeed(0, 0);
                    Robot2014.intakeClaw.setIntakeRollerOutput(0.7);
                }
                break;
//            
//            case 7: //back up, lower arm, close claw
//                if(!twoBall) {
//                    autonState = 0;
//                    break;
//                }
//                Robot2014.intakeClaw.setClawOpen(false);
//                Robot2014.robotArm.goToPreset(3);
//                if(driveToDistance((14-1.5)*12))
//                    autonState++;
//                break;
//            case 8:
//                if(timer.get() > .5)
//                    autonState++;
//                break;
//            
//            case 9: //pickup ball
//                Robot2014.intakeClaw.setIntakeRollerOutput(0.8);
//                if(driveToDistance(14*12))
//                    autonState++;
//                break;
//            case 10:
//                if(timer.get() > .5)
//                    autonState++;
//                break;
//            case 11: //raise arm and shoot
//                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
//                Robot2014.robotArm.goToPreset(1);
//                if(!Robot2014.intakeClaw.getClawOpen() && Robot2014.robotArm.nearSetpoint()) {
//                    Robot2014.shooterPunch.fire();
//                    timer.reset();
//                    autonState++;
//                } else {
//                    Robot2014.intakeClaw.setClawOpen(true);
//                }
//                break;
//            case 12:
//                if(timer.get() > .5)
//                    autonState++;
//                break;
//            case 13:
//                Robot2014.intakeClaw.setClawOpen(false);
//                autonState = 0;
//                break;
            case 0: default: //do nothing
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                return true;
        }
        return false;
    }
    
    private boolean oneBallMoving() {
        switch(autonState){
            //lower the robot arm until it hits the limit switch and zeros out
            case 1: //init loop
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                Robot2014.driveTrain.resetSensors();
                driveDistancePID.setMaxOutput(0.75);
                driveDistancePID.setMinOutput(-0.75);
                timer.reset();
                autonState++;
                break;
                
            case 2: //wait for sensors to reset, not sure how much time they need, 0.2 seems to be working
                if(timer.get() > 0.2)
                    autonState++;
                break;
                
            case 3: //drive to shooting location
                Robot2014.shooterPunch.setEnabled(true);
                Robot2014.intakeClaw.setClawOpen(false);
                if(driveToDistance(16.0*12)) {
                    if(Robot2014.driveTrain.getDistance()> 1.5*12)
                        Robot2014.robotArm.goToPreset(1);
                    if(Robot2014.driveTrain.getDistance()> 10*12 && 
                            Robot2014.robotArm.nearSetpointMoving()) {
                        Robot2014.robotArm.setMotorOutputManualAdjusted(0.0);
                        Robot2014.intakeClaw.setClawOpen(true);
                        Robot2014.shooterPunch.fire();
                        timer.reset();
                        autonState++;
                    }
                    timer.reset();
                    autonState++;
                }
                break;
                
            case 4: //wait for shot to finish
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                if(timer.get() > .5) {
                    Robot2014.intakeClaw.setClawOpen(false);
                    autonState = 0;
                }
                break;
                
            case 0: default: //do nothing
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                return true;
        }
        return false;
    }
    
    private boolean twoBallMoving() {
        
        switch(autonState){
            case 1: // prepare
                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                driveDistancePID.setMaxOutput(0.25);
                driveDistancePID.setMinOutput(-0.25);
                autonState++;
                break;
            case 2: // drive forward
                Robot2014.shooterPunch.setEnabled(true);
                Robot2014.intakeClaw.setClawOpen(false);
                Robot2014.robotArm.goToPreset(1);
                if(driveToDistance(11.5*12)){
                    Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
                }
                autonState++;
                break;
            case 3: // fire
                Robot2014.robotArm.setMotorOutputManualAdjusted(0.0);
                Robot2014.intakeClaw.setClawOpen(true);
                Robot2014.shooterPunch.fire();
                timer.reset();
                autonState++;
                break;
            case 4: // close after 1
                if(timer.get() > 1){
                    Robot2014.intakeClaw.setClawOpen(false);
                    timer.reset();
                    autonState++;
                    break;
                }
            case 5:
                if(driveToDistance(-1*12)){
                    Robot2014.robotArm.goToPreset(4);
                    Robot2014.intakeClaw.setIntakeRollerOutput(-1); // take ball in
                }
            case 6:
                while(Robot2014.intakeClaw.getBallRange() > 30){
                    Robot2014.driveTrain.setLeftRightSpeed(.25, .25);
                    Robot2014.intakeClaw.setIntakeRollerOutput(-1);
                }
                Robot2014.driveTrain.setLeftRightSpeed(0,0);
                Robot2014.intakeClaw.setIntakeRollerOutput(0);
                timer.reset();
                autonState++;
                break;
            case 7:
                if(timer.get()>2){
                    timer.reset();
                    Robot2014.robotArm.goToPreset(1);
                    autonState++;
                    break;
                }
            case 8:
                Robot2014.intakeClaw.setIntakeRollerOutput(0);
                Robot2014.intakeClaw.setClawOpen(true);
                Robot2014.shooterPunch.fire();
                timer.reset();
                autonState++;
                break;
                
            case 9:
                if(timer.get() > 1){
                    Robot2014.intakeClaw.setClawOpen(false);
                    timer.reset();
                    autonState=0;
                    break;
                }
            case 0: default:
                return true;
        }
        
        
        return true;
    }
    
    
    
    private boolean driveToDistance(double distance) {
        driveDistancePID.setSetPoint(distance);
//        driveAnglePID.setSetPoint();//for now just leave it whatever it was last, no way to read
//        double drivePIDOutput = -driveDistancePID.updateAndGetOutput(Math.abs(Robot2014.driveTrain.getDistance()));
        double drivePIDOutput = -driveDistancePID.updateAndGetOutput(Robot2014.driveTrain.getDistance());
        double anglePIDOutput = driveAnglePID.updateAndGetOutput(Robot2014.driveTrain.getAngle());
        SmartDashboard.putNumber("Auton drive PID output", drivePIDOutput);
        SmartDashboard.putNumber("Auton drive angle PID output", anglePIDOutput);
        Robot2014.driveTrain.setLeftRightSpeed(drivePIDOutput - anglePIDOutput, drivePIDOutput + anglePIDOutput);
        SmartDashboard.putNumber("Auton drive PID error", driveDistancePID.getError());
        SmartDashboard.putNumber("Auton drive angle PID error", driveAnglePID.getError());
        SmartDashboard.putNumber("Auton timer", timer.get());
        // code for both PID good for X time
//        if( !(Math.abs(driveDistancePID.getError())<1.5) || !(Math.abs(driveAnglePID.getError())<0.5)) {
//            timer.reset();
//        }
//        return timer.get() > 0.75;
        // code for both PID good + wheels stopped, should be faster than time
        return (Math.abs(driveDistancePID.getError())<0.5) && 
                (Math.abs(driveAnglePID.getError())<2.5) && 
                (Math.abs(Robot2014.driveTrain.getSpeed()) < 0.1);
    }
    
    // abt 41 angle units in 90 degrees
    private boolean driveToAngle(double angle) {
        //will stay from whatever last unless after a reset, so throw it in
//        driveDistancePID.setSetPoint(Math.abs(Robot2014.driveTrain.getDistance()));
        driveDistancePID.setSetPoint(Robot2014.driveTrain.getDistance());
        driveAnglePID.setSetPoint(angle);
//        double drivePIDOutput = -driveDistancePID.updateAndGetOutput(Math.abs(Robot2014.driveTrain.getDistance()));
        double drivePIDOutput = -driveDistancePID.updateAndGetOutput(Robot2014.driveTrain.getDistance());
        double anglePIDOutput = driveAnglePID.updateAndGetOutput(Robot2014.driveTrain.getAngle());
        // the min turn amonut the robot turns at is 0.09 * 12.8v either direction
        if(anglePIDOutput>0)
            anglePIDOutput +=0.075;
        else
            anglePIDOutput -=0.075;
        
        SmartDashboard.putNumber("Auton drive PID output", drivePIDOutput);
        SmartDashboard.putNumber("Auton drive angle PID output", anglePIDOutput);
        Robot2014.driveTrain.setLeftRightSpeed(drivePIDOutput - anglePIDOutput, drivePIDOutput + anglePIDOutput);
        SmartDashboard.putNumber("Auton drive PID error", driveDistancePID.getError());
        SmartDashboard.putNumber("Auton drive angle PID error", driveAnglePID.getError());
        SmartDashboard.putNumber("Auton timer", timer.get());
        // code for both PID good for X time
//        if( !(Math.abs(driveDistancePID.getError())<1.5) || !(Math.abs(driveAnglePID.getError())<0.5)) {
//            timer.reset();
//        }
//        return timer.get() > 0.75;
        // code for both PID good + wheels stopped, should be faster than time
        return (Math.abs(driveDistancePID.getError())<0.5) && 
                (Math.abs(driveAnglePID.getError())<2.5) && 
                (Math.abs(Robot2014.driveTrain.getSpeed()) < 0.1);
    }
//    private void drive Elims(){
//        switch(autonState){
//            case 1:
//                Robot2014.driveTrain.setLeftRightSpeed(0.0, 0.0);
//                Robot2014.driveTrain.resetSensors();
//                Robot2014.driveTrain.freeRangeGyro();
//                driveDistancePID.setMaxOutput(0.60);
//                driveDistancePID.setMinOutput(-0.60);
//                timer.reset();
//                autonState++;
//                break;  
//                
//            case 2: //wait for sensors to reset, not sure how much time they need, 0.2 seems to be working
//                if(timer.get() > 0.2)
//                    autonState++;
//                break;
//                
//            case 3: //drive to shooting location
//                if(driveToDistance(12.833333*12-2)) {
//                    timer.reset();
//                    autonState=0;
//                }
//                break;
//            case 0: break;
//        }
//    }
}

