package ca.mcgill.ecse211.wallfollowing;

//import lejos.hardware.motor.EV3LargeRegulatedMotor;
// The LEFT wheel is the OUTER wheel
//Can now pivot rather than reverse

//For some reason, P-controller over-compensates while attempting to take corners - I do not know how to fix it 

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 22;
  private static int outerWheel;
  private static int innerWheel;
  private static final int constant = 3;
  private static final int minDistance = 16;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter - 4; //changed band center from 30 to 22, increase was for bangbang controller
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 50 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 50) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 40: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    // TODO: process a movement based on the us distance passed in (P style)
    if(this.distance <= (bandCenter+bandWidth) && this.distance >= (bandCenter-bandWidth)) { //no correction needed, robot should move straight
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } else if(this.distance < (bandCenter-bandWidth) && this.distance < minDistance) { //robot is WAY too close to the wall - reverse/rotate until the distance is acceptable
    	/*int diff = Math.abs(distance - (bandCenter-bandWidth));
    	outerWheel = MOTOR_SPEED + (constant*diff);
    	innerWheel = MOTOR_SPEED - (constant*diff); //changed this...
    	WallFollowingLab.rightMotor.setSpeed(innerWheel/2);
    	WallFollowingLab.leftMotor.setSpeed(outerWheel);*/
    	int diff = Math.abs(distance - (bandCenter - bandWidth));
    	innerWheel = (2*(MOTOR_SPEED + (constant*diff)) + (constant*diff));
    	outerWheel = MOTOR_SPEED;
    	WallFollowingLab.rightMotor.setSpeed(innerWheel);
    	WallFollowingLab.leftMotor.setSpeed(outerWheel);
    	WallFollowingLab.rightMotor.forward();
    	WallFollowingLab.leftMotor.forward();
    } else if(this.distance < (bandCenter-bandWidth)) {//robot is still too close to the wall
    	int diff = Math.abs(this.distance - (bandCenter-bandWidth));
    	/*outerWheel = MOTOR_SPEED /*- (constant*diff); 
    	innerWheel = MOTOR_SPEED + (constant*diff);*/
    	outerWheel = MOTOR_SPEED + 2*(constant*diff); // this causes reversal:: MOTOR_SPEED - 2*(constant*diff);
    	innerWheel = MOTOR_SPEED /*this can work with pivoting - needs to be tweaked: - 2*(constant*diff)*/;
    	WallFollowingLab.leftMotor.setSpeed(outerWheel);
    	WallFollowingLab.rightMotor.setSpeed(innerWheel);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } else  { //robot is too far from the wall
    	int diff = Math.abs(this.distance - (bandCenter+bandWidth));
    	outerWheel = MOTOR_SPEED + 2*(constant*diff);
    	innerWheel = MOTOR_SPEED; //changed this...
    	WallFollowingLab.leftMotor.setSpeed(outerWheel);
    	WallFollowingLab.rightMotor.setSpeed(innerWheel);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
