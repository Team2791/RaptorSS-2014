/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package org.usfirst.frc2791.robot2014.subsystems;

import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 *
 * @author mwtidd
 */
public class ExceptionLogger {
    public static void logAlreadyInitalizedWarning(){
        System.out.println();
        System.out.println("[WARNING] The camera has already been initalized.");
        System.out.println("It will not be initialized again.");
        System.out.println();
    }
    
    public static void logNotInitalizedWarning(){
        System.out.println();
        System.out.println("[WARNING] The camera hasn't been initalized manually.");
        System.out.println("This can cause a delay in vision processing.");
        System.out.println("It it best to call the initalize function in robot init or command base init.");
        System.out.println("Initializing the camera autonmatically...");
        System.out.println();
    }
    
    public static void logGetImageException(String className){
        if(className.equals(AxisCameraException.class.getName())){
            System.out.println();
            System.err.println("[ERROR] Axis Camera Exception.");
            System.out.println("There was in issue getting the image from the camera");
            System.out.println();
        }else if(className.equals(NIVisionException.class.getName())){
            System.out.println();
            System.err.println("[ERROR] NI Vision Exception.");
            System.out.println("There was in issue getting the image from the camera");
            System.out.println();
        }
    }
    
    public static void logParticleFilterException(){
        System.out.println();
        System.err.println("[ERROR] NI Vision Exception.");
        System.out.println("There was an issue filtering the image");
        System.out.println();
    }
    
    public static void logParticleCountException(){
        System.out.println();
        System.err.println("[ERROR] NI Vision Exception.");
        System.out.println("There was an issue counting the particles in the image");
        System.out.println();
    }
    
    public static void logParticleReportExcepeiton(int particleNumber){
        System.out.println();
        System.err.println("[ERROR] NI Vision Exception.");
        System.out.println("There was an issue processing paticle #"+particleNumber);
        System.out.println();
    }
    
    public static void logFreeImageException(){
        System.out.println();
        System.err.println("[ERROR] NI Vision Exception.");
        System.out.println("There was an issue freeing the images.");
        System.out.println("This may cause a memory leak.");
        System.out.println("The hot detection may still return true in this case.");
        System.out.println();
    }
}
