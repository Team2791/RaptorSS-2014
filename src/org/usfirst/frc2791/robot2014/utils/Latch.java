package org.usfirst.frc2791.robot2014.utils;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author 2791
 */
public class Latch {
    private boolean output;
    private boolean firstSwitch = true;
    
    public Latch(boolean defaultState) { output = defaultState; }
    
    public void setLatchInput(boolean input) {
        if(input) {
            if(firstSwitch) { //first time the button was hit after being released
                output = !output;
                firstSwitch = false;
            } //otherwise do nothing
        } else { //butotn released
            firstSwitch = true;
        }
        
    }
   public void setManual(boolean newOutput) { output = newOutput; }
   public boolean getLatchOutput() { return output; }
   public boolean get() { return getLatchOutput(); } 
    
}
