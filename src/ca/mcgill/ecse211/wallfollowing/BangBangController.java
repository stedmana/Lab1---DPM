package ca.mcgill.ecse211.wallfollowing;

//import lejos.hardware.motor.*;
// The LEFT wheel is the OUTER wheel

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  public static final double PROPCONST = 1.0;
  
  
  private int distance;
  
  public static final int minDistance = 13;
  public static final int MAXCORRECTION = 50;
  public static final int FWDSPEED = 200;
  //public static final int WALLDIST = 20;
  public static final int MAXDIST = 200;
  public static final int ERRORTOL = 1;
  public static int distError = 0;
  public static int wallDist = 0;
  public static int leftSpeed;
  public static int rightSpeed;
  
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
    	//distError = WALLDIST - distance;
    //int diff;
    if(distance <= bandCenter+bandwidth && distance >= bandCenter-bandwidth) { //the error is within the bounds, no correction needed
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } else if(distance < bandCenter-bandwidth && distance < minDistance) { //Vehicle is way too close 
    	WallFollowingLab.leftMotor.setSpeed(2*motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorLow);
    	WallFollowingLab.leftMotor.backward();
    	WallFollowingLab.rightMotor.forward();
    } else if(distance < bandCenter-bandwidth) { //the vehicle is too close to the wall: move away!
    	WallFollowingLab.leftMotor.setSpeed(motorLow);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } else { //the vehicle is too far from the wall: move closer!
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorLow-5);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } 
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  public int calcProp(int diff) {
	  int correction;
	  diff = Math.abs(diff);
	  correction = (int) (PROPCONST * (double) diff);
	  if (correction >= FWDSPEED) {
		  correction = MAXCORRECTION;
	  }
	  return correction;
  }
}
