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
    
    private double m_deadZone;
    private double m_deadTime = 0.0;
    private Timer deadTimeTimer = new Timer();
    
    public FloppityPID(double p, double i, double d, double deadZone, double deadTime) {
        super(p, i, d);
        m_deadZone = deadZone;
        m_deadTime = deadTime;
        deadTimeTimer.start();
    }
    
    public void changeDeadzone(double deadZone) {
        m_deadZone = deadZone;
    }
    
    public double updateAndGetOutput(double currentValue) {
        double normalOutput = super.updateAndGetOutput(currentValue);
        if(deadTimeTimer.get() < m_deadTime) // if jut hit setpoint and dead time still counting return 0
            return 0;
        if(m_currentError < m_deadZone && m_currentError > -m_deadZone) {
            deadTimeTimer.reset();
            return 0;
        }
        else return normalOutput;
    }
    
}
