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
  public Timer sensorTimeoutTimer;
  //0 top, 1 mid, 2 bot
  private static short lastSensorHit;
  //the amount of time in seconds to wait before starting to pull back the
  //shooter again after a shot
  public final double SHOT_TIME = 1.0;
  private static boolean pullingBack = false;
  //sometimes after the bottom sensor trips and the punch stops pulling back it backdrives and no sensors
  //are hit, this is to let us still fire when that happens
  private static boolean backdrive_cludge = false;
  //    private static boolean LOWERING = true;

  public ShooterPunch() {
    windingMotor = new Victor(1, 5);
    releaseSolenoid = new DoubleSolenoid(3, 4);
    setReleaseSolonoid(false);

    sensorTimeoutTimer = new Timer();
    sensorTimeoutTimer.start();

    topSensor = new DigitalInput(8);
    midSensor = new DigitalInput(7);
    botSensor = new DigitalInput(6);
    botSensorBackup = new DigitalInput(9);
    readSensors();
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
    SmartDashboard.putBoolean("Mechanical Backdrive Cludge", backdrive_cludge);
    //first thing first, if the shooter is not loaded load it
    if (fire) { //if time to fire, well FIRE
      setReleaseSolonoid(true);
      sensorTimeoutTimer.reset();
      pullingBack = false;
    }
    if (enabled) {
      if (!readyToFire()) {
        //check if we are in a cooldown period, en
        //                if(shotTimer.get() < SHOT_TIME) {
        if (pullingBack || !topSensor.get()) { //if hit the top sensor or after hit and still pulling
          pullingBack = true;
          setReleaseSolonoid(false);
          fire = false;
          //                    windingMotor.set(-0.8);
          if (lastSensorHit == 0) //top sensor
          {
            setMotorSpeed(1.0);
          } else if (lastSensorHit == 1) //mid sensor
          {
            setMotorSpeed(0.50); //was -.80 then -.50
          }
        } else { //waiting for punch to hit top sensor

          //wait and do nothing
          setMotorSpeed(0);
        }
      } else { //shooter is ready to fire
        pullingBack = false;
        //let us fire after we hit this stage, even if sensors aern't still tripped
        backdrive_cludge = true;
        setMotorSpeed(0); //turn off the winding motor
      }
    } else { //disabled
      setMotorSpeed(0);
    }
  }

  private void setMotorSpeed(double speed) {
    if (speed < 0) {
      speed = 0;
    }
    if (sensorTimeoutTimer.get() > 3.0) {
      windingMotor.set(0);
    } else {
      windingMotor.set(-speed);
    }
  }

  private void readSensors() {
    if (!topSensor.get()) {
      lastSensorHit = 0;
    } else if (!midSensor.get()) {
      lastSensorHit = 1;
    } else if (!botSensor.get() || !botSensorBackup.get()) {
      lastSensorHit = 2;
    }
  }

  private void setReleaseSolonoid(boolean release) {
    if (release) {
      releaseSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      releaseSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public boolean readyToFire() {
    return !botSensor.get() || !botSensorBackup.get() || backdrive_cludge;
    //        return !midSensor.get();
  }

  //this method fires if able and returns if it was able to fire or not
  public boolean fire() {
    if (readyToFire()) {
      backdrive_cludge = false; //don't let us fire twice
      fire = true;
      return true;
    } else {
      return false;
    }
  }

  public void fireOverride() {
    fire = true;
  }

  public void disable() {
    backdrive_cludge = false;
    SmartDashboard.putBoolean("Mechanical Backdrive Cludge", backdrive_cludge);
    fire = false;
    setEnabled(false);
    sensorTimeoutTimer.reset();
    readSensors();
  }

  public String getDebugString() {
    //will replace this with an actual method when we know what the sensor is
    String dbgStr = "Ready: ";
    if (readyToFire()) {
      dbgStr += "Y ";
    } else {
      dbgStr += "N ";
    }
    dbgStr += "LstSenHit: " + lastSensorHit;
    return dbgStr;
  }

  public void setMotor(double set) {windingMotor.set(set);}

  public void display() {
    SmartDashboard.putBoolean("Top Sensor", !topSensor.get());
    SmartDashboard.putBoolean("Mid Sensor", !midSensor.get());
    SmartDashboard.putBoolean("Bot Sensor (6)", !botSensor.get());
    SmartDashboard.putBoolean("Bot Sensor Backup (9)", !botSensorBackup.get());
  }
}