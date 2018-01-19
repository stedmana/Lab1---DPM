package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  public static final double PROPCONST = 1.0;
  
  private int distance;
  
  public static final int MAXCORRECTION = 50;
  public static final int FWDSPEED = 100;
  public static final int WALLDIST = 30;
  public static final int MAXDIST = 200;
  public static final int ERRORTOL = 1;
  public static int distError = 0;
  public static int wallDist = 0;
  public static int leftSpeed;
  public static int rightSpeed;
  
  
  
  static Port portUS = LocalEV3.get().getPort("S1");
  static SensorModes myUS = new EV3UltrasonicSensor(portUS);
  static SampleProvider myDistance = myUS.getMode("Distance");
  static float[] sampleUS = new float[myDistance.sampleSize()];
  
  //test comment

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    
  }

  @Override
  public void processUSData(int distance) {
	  
	int diff;
    this.distance = distance;
    myDistance.fetchSample(sampleUS, 0);
    wallDist = (int)(sampleUS[0] * 100.0);
    
    if(wallDist <= MAXDIST) {
    	distError = WALLDIST - wallDist;
    }
    
    if(Math.abs(distError) <= ERRORTOL) { //the error is within the bounds, no correction is required
    	leftSpeed = FWDSPEED;
    	rightSpeed = FWDSPEED;
    	WallFollowingLab.leftMotor.setSpeed(leftSpeed);
    	WallFollowingLab.rightMotor.setSpeed(rightSpeed);
    } else if (distError > 0) { //the vehicle is too close to the wall: move away!
    	diff = calcProp(distError);
    	leftSpeed = FWDSPEED + diff;
    	rightSpeed = FWDSPEED - diff;
    	WallFollowingLab.leftMotor.setSpeed(leftSpeed);
    	WallFollowingLab.rightMotor.setSpeed(rightSpeed);
    } else { //the vehicle is too far from the wall: move closer!
    	diff = calcProp(distError);
    	leftSpeed = FWDSPEED - diff;
    	rightSpeed = FWDSPEED + diff;
    	WallFollowingLab.leftMotor.setSpeed(leftSpeed);
    	WallFollowingLab.rightMotor.setSpeed(rightSpeed);
    }
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  public int calcProp(int diff) {
	  int correction;
	  diff = Math.abs(diff);
	  correction = (int) (PROPCONST * (double)diff);
	  if(correction >= FWDSPEED) {
		  correction = MAXCORRECTION;
	  }
	  return correction;
  }
}
