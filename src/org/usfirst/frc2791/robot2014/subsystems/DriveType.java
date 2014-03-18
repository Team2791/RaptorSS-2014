/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014.subsystems;

import com.sun.squawk.util.MathUtils;

public class DriveType {
    private boolean linear = false;
    private boolean slow = false;
    
    public DriveType(String type){
        if(type.equals("Linear")){
            linear = true;
        }
        if(type.equals("Slow")){
            slow = true;
        }
    }
    public double getJoystickValue(double input){
        return scaleJoysticks(input);
    }
    
    private double scaleJoysticks(double input){
        double vals[] = {0,0,0,0,0};
        if(this.slow){
            if(input==0)
                return 0;
            else if(input<0)
                return -scaleJoysticks(-input);
            else{
                vals[0] = MathUtils.pow(input,5);
                vals[0] *= 0;
            
                vals[1] = MathUtils.pow(input,4);
                vals[1] *= 5.1303;
            
                vals[2] = MathUtils.pow(input,3);
                vals[2] *= -7.6986;
            
                vals[3] = MathUtils.pow(input,2);
                vals[3] *= 3.6962;
            
                vals[4] *= input;
                vals[4] *= 0.133;
            
                return vals[0] + vals[1] + vals[2] + vals[3] + vals[4];
            }
            
        }else if(this.linear)
            return input;
        else
            return input;
    }
}


//    public double scaleJoysticks(double value, String type){
//        double newVal = 1;
//        double[] vals = {1,1,1,1,1};
//       
//        // current equation:
//        // y = 0.9703x^5 - 2.673x^4 + 0.3901x^3 + 2.0981x^2 + 0.2075x
//        
//        if(type.equals("Accelerate")){
//            if(value==0)
//                return 0;
//            else if(value>0.956)
//                return 1;
//            else if(value<-0.956)
//                return -1;
//            else if(value<0){
//                return -scaleJoysticks(-value, type);
//            }
//            else{
//                vals[0] = MathUtils.pow(value,5);
//                vals[0] *= 0.9703;
//            
//                vals[1] = MathUtils.pow(value,4);
//                vals[1] *= -2.673;
//            
//                vals[2] = MathUtils.pow(value,3);
//                vals[2] *= 0.3901;
//            
//                vals[3] = MathUtils.pow(value,2);
//                vals[3] *= 2.0981;
//            
//                vals[4] *= value;
//                vals[4] *= 0.9703;
//            
//                newVal = vals[0] + vals[1] + vals[2] + vals[3] + vals[4];
//            }
//        
//        
//            if(newVal > 1)
//                return 1;
//            if(newVal < -1)
//                return -1;
//        }
//        
//        
//        return newVal;
//    }
