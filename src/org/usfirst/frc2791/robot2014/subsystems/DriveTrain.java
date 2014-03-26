/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.Robot2014;
/**
 *
 * @author Maxwell
 */
public class DriveTrain extends Team2791Subsystem {
    //robot parts
    private static SpeedController driveTrainLeftMotorController;
    private static SpeedController driveTrainRightMotorController;
    //these should be made private and getter methods added
    private static Encoder driveTrainEncoderLeft;
    private static Encoder driveTrainEncoderRight;
    private static Gyro gyro;
    private static boolean gyroCal = false;
    private DriveType driveType, slow = new DriveType("Slow"), linear = new DriveType("Linear");
    //constants
    private static final boolean RIGHT_SIDE_REVERSED = false;
    
    
    public DriveTrain() {
         System.out.println("Running Drivetrain init");
        //init motors
        //MAKE SURE TO CHANGE THESE TO TALLONS WHEN THEY ARE CHANGED!!!!
        driveTrainLeftMotorController = new Talon(1, 1);
	LiveWindow.addActuator("Drive Base", "Drive Right 1", (Talon) driveTrainLeftMotorController);
        driveTrainRightMotorController = new Talon(1, 2);
	LiveWindow.addActuator("Drive Base", "Drive Right 1", (Talon) driveTrainRightMotorController);

        //init encoders
        //Encoder(int aChannel, int bChannel, boolean reverseDirection, CounterBase.EncodingType encodingType)
        driveTrainEncoderLeft = new Encoder(4, 5, RIGHT_SIDE_REVERSED, CounterBase.EncodingType.k4X);
        driveTrainEncoderRight = new Encoder(2, 3, !RIGHT_SIDE_REVERSED, CounterBase.EncodingType.k4X);
        //diamater of the wheel * pi = distance per rotation
        // 1 / ticks per rotation = rotations per tick
        //distance per rotation * rotations per tick = distance per tick
        //diamater of wheel * pi / ricks per rotation = distance per tick
        //4'' * pi / 128 = distance per tick
        double distancePerTick = 4 * Math.PI / 128; 
        driveTrainEncoderLeft.setDistancePerPulse(distancePerTick);
        driveTrainEncoderRight.setDistancePerPulse(distancePerTick);
        driveTrainEncoderLeft.start();
        driveTrainEncoderRight.start();
        
        gyro = new Gyro(2);
        
        driveType = linear;
    }
    
    public void setLeftRightSpeed(double leftSpeed, double rightSpeed) {
        rightSpeed *= 12.8;
        leftSpeed *= 12.8;
        //calculate % of battery voltage the target output voltage is
        rightSpeed /= Robot2014.station.getBatteryVoltage();
        leftSpeed /= Robot2014.station.getBatteryVoltage();
        
        if(!RIGHT_SIDE_REVERSED)
            rightSpeed = -rightSpeed;
        else
            leftSpeed = -leftSpeed;
        driveTrainLeftMotorController.set(leftSpeed);
        driveTrainRightMotorController.set(rightSpeed);
        
//        driveTrainLeftMotorController.set(driveType.getJoystickValue(leftSpeed));
//        driveTrainLeftMotorController.set(driveType.getJoystickValue(rightSpeed));
        // test soon........

        
        
    }
    
     /**
     * This is the code that runs in disabled, for safety
     */
    public void disable() {
        driveTrainLeftMotorController.set(0.0);
        driveTrainRightMotorController.set(0.0);
        driveTrainEncoderLeft.reset();
        driveTrainEncoderRight.reset();
    }

    public void run() {
        //there currently is no code in run, if we wanted the drive train to hold a
        //constant speed this is where we would do it
        
    }
    
    public void display(){
        SmartDashboard.putNumber("DT Encoder Left", driveTrainEncoderLeft.getDistance());
        SmartDashboard.putNumber("DT Encoder Right", driveTrainEncoderRight.getDistance());
        SmartDashboard.putNumber("Encoder Avg Val", (getDistance()));
        SmartDashboard.putBoolean("Drive Safety", Robot2014.teleopRunner.SAFETY_MODE);
        SmartDashboard.putString("Drive Type", Robot2014.teleopRunner.DRIVE_TYPE);
        SmartDashboard.putNumber("DT Encoder Left Speed", driveTrainEncoderRight.getRate());
        SmartDashboard.putNumber("DT Encoder Right Speed", driveTrainEncoderRight.getRate());
        SmartDashboard.putNumber("DT Encoder Avg Speed", getSpeed());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putBoolean("Gyro Calibrated!!", gyroCal);
        
    }
    
    // driveTrain.setLeftRightSpeed(scaleJoysticks(joystickLeft.getY()), scaleJoysticks(joystickRight.getY()));
    

    
    public double getSpeed() {
//        return (driveTrainEncoderLeft.getRate() + driveTrainEncoderRight.getRate())/2;
        return driveTrainEncoderRight.getRate();
    }
    
    public double getDistance() {
//        return (driveTrainEncoderLeft.getDistance() + driveTrainEncoderRight.getDistance())/2;
        return driveTrainEncoderRight.getDistance();
    }
    
    public double getTotalJoystickInput(){
        return Robot2014.driverRightStick.getY() + Robot2014.driverLeftStick.getY();
    }
    
    public double getEncoderDiff() { //left - right in inches, when this is + we have turned CW
        return driveTrainEncoderLeft.getDistance() - driveTrainEncoderRight.getDistance();
    }
    
    public double getAngle(){
        //could also use encoder diff and a constant, wouldn't be as accurate though
        return gyro.getAngle();
    }
    
    public void resetSensors() {
        driveTrainEncoderLeft.reset();
        driveTrainEncoderRight.reset();
//        gyro.free();
//        gyro = new Gyro(2);
        gyro.reset();
    }
    
    //takes six seconds
    public void freeRangeGyro() {
        gyro.free();
        gyro = new Gyro(2);
        gyroCal = true;
    }
    
    public void setDriveType(DriveType type){
        driveType = type;
    }
}