package ca.mcgill.ecse211.wallfollowing;

//import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static int outerWheel = 200;
  private static final int constant = 7;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
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
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    // TODO: process a movement based on the us distance passed in (P style)
    if(distance <= bandCenter+bandWidth && distance >= bandCenter-bandWidth) { //no correction needed, robot should move straight
    	//WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	//WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    } else if(distance < bandCenter-bandWidth) { //robot is too close to the wall
    	int diff = Math.abs(distance - (bandCenter-bandWidth));
    	outerWheel = MOTOR_SPEED - (constant*diff);
    	WallFollowingLab.rightMotor.setSpeed(outerWheel);
    	WallFollowingLab.rightMotor.forward();
    } else if(distance > bandCenter+bandWidth) { //robot is too far from the wall
    	int diff = Math.abs(distance - bandCenter+bandWidth);
    	outerWheel = MOTOR_SPEED + (constant*diff);
    	WallFollowingLab.rightMotor.setSpeed(outerWheel);
    	WallFollowingLab.rightMotor.forward();
    }
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
