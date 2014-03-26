/* Possible joystick to PWM equations:
 * y = x^3
 * y = 2x^2
 * y = 0.9703x^5 - 2.673x^4 + 0.3901x^3 + 2.0981x^2 + 0.2075x
 * y = 5.1303x^4 - 7.6986x^3 + 3.6962x^2 - 0.133x
 */

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import org.usfirst.frc2791.robot2014.subsystems.DriveTrain;
import org.usfirst.frc2791.robot2014.subsystems.ShooterPunch;
import org.usfirst.frc2791.robot2014.subsystems.RobotArm;
import org.usfirst.frc2791.robot2014.subsystems.IntakeClaw;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2791.robot2014.AutonRunner;
import org.usfirst.frc2791.robot2014.TeleopRunner;
//import org.usfirst.frc2791.robot2014.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot2014 extends IterativeRobot {
    
    
    
    
    // encoders
    public static Encoder rightDriveEncoder;
    public static Encoder leftDriveEncoder;
    
    //robot systems
    public static DriveTrain driveTrain;
    public static Compressor compressor;
    public static RobotArm robotArm;
    public static IntakeClaw intakeClaw;
    public static ShooterPunch shooterPunch;
//    public static HotCamera camera = new HotCamera();
    //opereator interface
    public static Joystick driverLeftStick;
    public static Joystick driverRightStick;
    public static Joystick operatorStick;
    //auton and teleop runner classes
    public static AutonRunner autonRunner;
    public static TeleopRunner teleopRunner;
    //for printing to the driver station
    public static SmartDashboard dash;
    public static DriverStationLCD ds_lcd;
    public static DriverStation station;
    public static Preferences prefs;
    
    public static final String SPACE = "                   ";
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        SmartDashboard.putString("Robot code version", "1.1.0");
        
        System.out.println("Running robotInit");
        
        dash = new SmartDashboard();
        ds_lcd = DriverStationLCD.getInstance();
        station = DriverStation.getInstance();
//        camera.initialize();
        
        prefs = Preferences.getInstance();
        //init robot things
        driveTrain = new DriveTrain();
        robotArm = new RobotArm();
        compressor = new Compressor(1, 1);
        intakeClaw = new IntakeClaw();
        //init joysticks
        driverLeftStick = new Joystick(1);
        driverRightStick = new Joystick(2);
        operatorStick = new Joystick(3);
        //init runner classes
        autonRunner = new AutonRunner();
        teleopRunner = new TeleopRunner();
        shooterPunch = new ShooterPunch();
        //dash stuff
        
        System.out.println("Finished robotInit");
        
        
    }
    
    //this gets called before autonomus periodic is called for the first time 
    //or for the firs time after soemthing else is called
    public void autonomousInit() {
        System.out.println("Auton init called");
        
        //reset the runner so we always start at the begining of auton
        autonRunner.reset();
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
//        System.out.println("auto periodic called");
        autonRunner.run();
        runAllSubsystems();
    }

    public void teleopInit() {
        teleopRunner.reset();
    }
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
//        System.out.println("teleop periodic called");
        teleopRunner.run();
        runAllSubsystems();
//        System.out.println("Hot Goal Found: "+camera.isHot());
    }
    
    
    //    public void disabledInit() {}
    public void disabledPeriodic() {
        //System.out.println("disabled periodic called");
        display();
        driveTrain.disable();
        robotArm.disable();
        intakeClaw.disable();
        shooterPunch.disable();
        
        compressor.stop();
        
        teleopRunner.setSafety(true);
        //when driver holds LB and RB recal the gyro, this is gonna cause some lag but that's okay for now
        if(Robot2014.driverLeftStick.getRawButton(5) && Robot2014.driverLeftStick.getRawButton(6)) {
            SmartDashboard.putBoolean("gyro calibrating", true);
            driveTrain.freeRangeGyro();
            SmartDashboard.putBoolean("gyro calibrating", false);
        }
    }
    
    //    public void testInit() {}
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        
    }
    
    public void runAllSubsystems(){
//        System.out.println("run all subsystems called");
//        driveTrain currently has no code to run in the background at the momment
        
        driveTrain.run();
        robotArm.run();
        intakeClaw.run();
        //shooterPunch.run is production code w/ sensors, shooterPunch.test is manual control
        shooterPunch.run();
//        shooterPunch.test();
        
        display();
    }
    
    public void disabledInit(){

    }
    
    public void display(){
        SmartDashboard.putNumber("Target Error",robotArm.armPID.getError());
        
//        compressor is automatically run and stopped
    //ds_lcd.println(DriverStationLCD.Line.kUser1,1,"LeftJoy: "+driverLeftStick.getY()+SPACE);
        //ds_lcd.println(DriverStationLCD.Line.kUser2,1,"RightJoy: "+driverRightStick.getY()+SPACE);
        
        
//        ds_lcd.println(DriverStationLCD.Line.kUser1,1,"right enc: "+Math.abs(DriveTrain.driveTrainEncoderRight.getDistance())+SPACE);
//        ds_lcd.println(DriverStationLCD.Line.kUser2,1,"left enc: "+Math.abs(DriveTrain.driveTrainEncoderLeft.getDistance())+SPACE);
//        ds_lcd.println(DriverStationLCD.Line.kUser4,1,"OP Stick: "+operatorStick.getY()+SPACE);
        
        ds_lcd.println(DriverStationLCD.Line.kUser3,1,robotArm.getDebugString()+SPACE);
        ds_lcd.println(DriverStationLCD.Line.kUser4,1,intakeClaw.getDebugString()+SPACE);
        ds_lcd.println(DriverStationLCD.Line.kUser5,1,shooterPunch.getDebugString()+SPACE);
        ds_lcd.updateLCD();
        
        driveTrain.display();
        robotArm.display();
        intakeClaw.display();
        shooterPunch.display();
    }
    
    public static double getDoubleAutoPut(String key, double default_value) {
        double val = prefs.getDouble(key,default_value);
        if(val == default_value)
            prefs.putDouble(key,default_value);
        return val;
    }
    
    public static boolean getBoolAutoPut(String key, boolean default_value) {
        boolean val = prefs.getBoolean(key,default_value);
        if(val == default_value)
            prefs.putBoolean(key,default_value);
        return val;
    }
        
}
