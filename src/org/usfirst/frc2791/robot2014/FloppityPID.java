/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2791.robot2014;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author 2791
 */
public class FloppityPID extends BasicPID {
    
    private double m_deadZone = 0.0;
    
    public FloppityPID(double p, double i, double d, double deadZone) {
        super(p, i, d);
        m_deadZone = deadZone;
    }
    
    public void changeDeadzone(double deadZone) {
        m_deadZone = deadZone;
    }
    
    public double updateAndGetOutput(double currentValue) {
        double normalOutput = super.updateAndGetOutput(currentValue);
        if(m_currentError < m_deadZone && m_currentError > -m_deadZone)
            return 0;
        else return normalOutput;
    }   
}
