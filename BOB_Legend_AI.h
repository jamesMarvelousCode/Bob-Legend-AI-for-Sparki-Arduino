//Bob Legend AI for Sparki Arduino
//Authors: James Dotson & Ben Robinson
//10/20/2016
#include <Sparki.h>
#include "pitches.h"

int HUNDREDTH_SECOND_DELAY = 10;
int FOURTIETH_SECOND_DELAY = 25;
int TWENTIETH_SECOND_DELAY = 50;
int TENTH_SECOND_DELAY = 100;
int FIFTH_SECOND_DELAY = 200;
int TWO_SECOND_DELAY = 2000;
int THREE_SECOND_DELAY = 3000;
int TEN_SECOND_DELAY = 10000;

int LOOK_SLIGHT_RIGHT = 30;
int LOOK_SLIGHT_LEFT = -30;
int LOOK_RIGHT = 45;
int LOOK_LEFT = -45;
int LOOK_FULL_RIGHT = 90;
int LOOK_FULL_LEFT = -90;

int SLIGHT_TURN = 25;
int EDGE_TURN = 30;
int TURN = 45;
int FULL_TURN = 90;
int EDGE_TURNAROUND = 100;
int TURNAROUND = 180;

int count = 0;
int SETUP_OPEN_GRIPPER = 3;
int SETUP_CLOSE_GRIPPER = 7;
int SETUP_PIN_READ = 0;
int CM_ERROR = -1;

int RANUM = 0;
int RAN_MIN = 1;
int RAN_MAX = 100;

int PICK_ROUTE_MIN_TRIGGER = 20;
int PICK_ROUTE_LONG_TRIGGER = 40;

int CM_TOO_CLOSE = 15;

int RANDOM_ADJUST = 20;
int SMALL_BACKUP = 1;

int NO_FLOOR = 200;
int COUNT_THRESHOLD = 2;
int SMALL_RANDOM_ADJUST = 6;

int HALF_RANUM = 49;
int SUM_RANUM = 20;

int CM_TOO_FAR = 30;

int CM_OBJECT_THRESHOLD = 10;
int SPIN = 360;

int LOOK_CENTER = 0;
int LOOK_RIGHT_MAX = 50;
int LOOK_LEFT_MAX = -50;
int LOOP_INCREMENT = 2;

int DELAYINT_INCREMENT = 100;
int GRIPPER_MOVE_CM = 2;

int LOOP_TIMED_SPIN = 72;
int BEEP_MULIPLIER = 10;
int BEEP_MAX_RANGE = 100;
	
//----------------------------------------------------------------------------------------------------------
//	void setup() these instructions fire one time at startup. Any instructions that need to be executed before the program starts go here.
// This includes a cosmetic starup sequence; just for funs.
//---------------------------------------------------------------------------------------------------------	
void setup()
{
	happyBeep();
	sparki.gripperClose(SETUP_CLOSE_GRIPPER);
	delay(TEN_SECOND_DELAY);
	sadBeep();
	sparki.gripperOpen(SETUP_OPEN_GRIPPER);
	delay(THREE_SECOND_DELAY);
	lookAbout();
	sparki.servo(SERVO_CENTER);
	sparki.clearLCD();
	randomSeed(analogRead(SETUP_PIN_READ));
	randomDirection();
}

//----------------------------------------------------------------------------------------------------------
// void loop() this is the main method. The nature of robot programming, similar to an operating system, requires instructions to be executed continually in a loop. Calls to the main decision making methods go here.
//----------------------------------------------------------------------------------------------------------
void loop()
{
	RANUM = random(RAN_MIN, RAN_MAX);//Random number seed for each loop to control random behaviors, add variance to several movement variables, and with a %2 function for a 50/50 chance
	
	sparki.RGB(RGB_GREEN);
    sparki.moveForward(); // start BOB moving forward

	pickRoute(); //teh majics
	
	behaviorsGO();//fires a behavior algorithm if the right seed was generated
	
    delay(TWENTIETH_SECOND_DELAY);
}

//----------------------------------------------------------------------------------------------------------
//	int pickRoute() this method fires at the start of each loop OR when BOB sees an object ahead. When this method fires, BOB will glance right and left to decide which way he should go.
//	RED
//----------------------------------------------------------------------------------------------------------
void pickRoute()
{
	int cmAhead = 0, routeDecision = 0, leftPing = 0, rightPing = 0;
	
	sparki.println("pickRoute()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	//call edge detection
	edgeDetection();
	
    cmAhead = sparki.ping(); // BOB's primary ping
	
	if(cmAhead != CM_ERROR) // make he gets a valid reading
    { 
		if (cmAhead <= PICK_ROUTE_MIN_TRIGGER)//far off obstacle detection
		{
			sparki.RGB(RGB_RED);
			//begin looking for a better route
			//centering servo before scan
			sparki.servo(SERVO_CENTER);
			
			//glance right and grab a ping
			sparki.servo(LOOK_SLIGHT_RIGHT);
			delay(TENTH_SECOND_DELAY);
			rightPing = sparki.ping();
			delay(TENTH_SECOND_DELAY);
			
			//glance left and grab a ping
			sparki.servo(LOOK_SLIGHT_LEFT);
			delay(TENTH_SECOND_DELAY);
			leftPing = sparki.ping();
			delay(TENTH_SECOND_DELAY);
			
			//re-centering servo after scan
			sparki.servo(SERVO_CENTER);
			
			//decide which is the better option
			//0 default take no new action
			//1 swerve left
			//2 swerve right
			//3 call pickDirection()
			
			if (leftPing <= PICK_ROUTE_MIN_TRIGGER && rightPing <= PICK_ROUTE_MIN_TRIGGER)
			{
				//BOBs sensors have found that he is appraching a large obstacle and will likely need to stop and find a new path
				routeDecision = 3;
			}
			else if (leftPing < rightPing)
			{
				routeDecision = 2;
			}
			else if (leftPing >= rightPing)
			{
				routeDecision = 1;
			}
			else if (leftPing <= PICK_ROUTE_LONG_TRIGGER && rightPing <= PICK_ROUTE_LONG_TRIGGER)
			{
				//if left and right pings are both under PICK_ROUTE_LONG_TRIGGER pickRoute() is called recursively to re-check
				pickRoute();
			}
			else
			{
				//send 0 to smallAdjustment() (re-fire pickRoute()) because something has probably gone wrong; most likely ping result inaccuracy has spit out some ugly numbers
				routeDecision = 0;
			}
			
			//send the decision to smallAdjustment()
			smallAdjustment(routeDecision);
		}
    }
}

//----------------------------------------------------------------------------------------------------------
//	void smallAdjustment() call this method and send it the decision int generated by void pickRoute()
//	RED
//----------------------------------------------------------------------------------------------------------
void smallAdjustment(int decision)
{
	sparki.println("smallAdjustment()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	sparki.RGB(RGB_VIOLET);
	switch(decision)
	{
		case 0:
			//pickRoute() gets called here because ALL previous methods failed to make a decision somehow
			pickRoute();
			break;
		case 1:
			sparki.moveStop();
			sparki.moveLeft(SLIGHT_TURN);
			delay(TENTH_SECOND_DELAY);
			sparki.moveForward();
			break;
		case 2:
			sparki.moveStop();
			sparki.moveRight(SLIGHT_TURN);
			delay(TENTH_SECOND_DELAY);
			sparki.moveForward();
			break;
		case 3:
			//BOBs sensors have found that he is appraching a large obstacle and will likely need to stop and find a new path
			pickDirection();
			break;
		default:
			//error action
			sadBeep();
			break;
	}
}

//----------------------------------------------------------------------------------------------------------
// void pickDirection() this method fires if BOB finds himself within 20cm of an object; most likely he has found himself in a corner. Once this method fires, BOB will perform a more detailed sweep of his surroundings and make a decision bassed on what he sees.
//	RED
//----------------------------------------------------------------------------------------------------------
void pickDirection()
{   
	sparki.println("pickDirection()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();

	//pick variables
	int tempPing = 0, decision = 0, rightPing = 0, fullRightPing = 0, leftPing = 0, fullLeftPing = 0;
	
	//centering servo before scan
	sparki.servo(SERVO_CENTER);

	//initial instructions before detailed scan. Remember, if these instructions execute, BOB has found himsef in a corner and needs to stop and look about to find his way
	sparki.moveStop();
	delay(TWENTIETH_SECOND_DELAY);    
      
	//get right ping
	sparki.servo(LOOK_RIGHT);
	delay(TENTH_SECOND_DELAY);
	rightPing = sparki.ping();
	
	//get full right ping
	sparki.servo(LOOK_FULL_RIGHT);
	delay(TENTH_SECOND_DELAY);
	fullRightPing = sparki.ping();

	//get left ping
	sparki.servo(LOOK_LEFT);
	delay(FIFTH_SECOND_DELAY);
	leftPing = sparki.ping();
      
	//get full left ping
	sparki.servo(LOOK_FULL_LEFT);
	delay(TENTH_SECOND_DELAY);
	fullLeftPing = sparki.ping();
	
	//re-centering servo after scan
	sparki.servo(SERVO_CENTER);

	//make decisions for 5 different outcomes
	//1 = turn Right
	//2 = full Right
	//3 = turn Left
	//4 = full Left
	//5 = turn around
	if (leftPing >= rightPing)
	{
		tempPing = leftPing;
		decision = 3;
	}
	else
	{
		tempPing = rightPing;
		decision = 1;
	}
	if (fullLeftPing > tempPing)
	{
		tempPing = fullLeftPing;
		decision = 4;
	}
	if (fullRightPing >= tempPing)
	{
		tempPing = fullRightPing;
		decision = 2;
	}
	if (rightPing <= CM_TOO_CLOSE && leftPing <= CM_TOO_CLOSE && fullRightPing <= CM_TOO_CLOSE && fullLeftPing <= CM_TOO_CLOSE)
	{
		decision = 5;
	}
	
	largeAdjustment(decision);
}

//----------------------------------------------------------------------------------------------------------
// void largeAdjustment() call this method and send it the decision int generated by void pickDirection()
//	VIOLET
//----------------------------------------------------------------------------------------------------------
void largeAdjustment(int decision)
{
	sparki.println("largeAdjustment()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	if (RANUM > RANDOM_ADJUST)
	{
		if (RANUM%2 == 0)// 50/50 if statement
		{
			TURNAROUND -= RANUM;
		}
		else
		{
			TURNAROUND += RANUM;
		}
	}
	
	sparki.RGB(RGB_VIOLET);
	switch(decision)
	{
		case 0:
			//pickRoute() gets called here because ALL previous methods failed to make a decision some how
			pickRoute();
			break;
		case 1:
			sparki.moveBackward(SMALL_BACKUP);
			sparki.moveRight(TURN);
			sparki.moveForward();
			break;
		case 2:
			sparki.moveBackward(SMALL_BACKUP);
			sparki.moveRight(FULL_TURN);
			sparki.moveForward();
			break;
		case 3:
			sparki.moveBackward(SMALL_BACKUP);
			sparki.moveLeft(TURN);
			sparki.moveForward();
			break;
		case 4:
			sparki.moveBackward(SMALL_BACKUP);
			sparki.moveLeft(FULL_TURN);
			sparki.moveForward();
			break;
		case 5:
			sparki.moveBackward(SMALL_BACKUP);
			sparki.moveRight(TURNAROUND);
			sparki.moveForward();
			break;
		default:
			//error action
			sadBeep();
			break;  
	}
}

//----------------------------------------------------------------------------------------------------------
// void edgeDetection() this method is called at the beginning of pickDirection to find edges
// this method will ++Count to keep track of how many times bob detects the same edge. if count gets too high he turns around. also if RANUM is low enough it is used to give the turnAround variable a bit of variance
//WHITE
//----------------------------------------------------------------------------------------------------------
void edgeDetection()
{
	sparki.println("edgeDetection()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	int edgeLeft   = sparki.edgeLeft();
	int edgeRight  = sparki.edgeRight();
	
	if (edgeLeft < NO_FLOOR) // if no surface underneath left sensor
	{
		sparki.RGB(RGB_WHITE);
		
		if (count > COUNT_THRESHOLD)
		{
			if (RANUM > SMALL_RANDOM_ADJUST)
			{
				if (RANUM%2 == 0)// 50/50 if statement
				{
					EDGE_TURNAROUND -= RANUM;
					sparki.moveRight(EDGE_TURNAROUND);
					count = 0;
				}
				else
				{
					EDGE_TURNAROUND += RANUM;
					sparki.moveRight(EDGE_TURNAROUND);
					count = 0;
				}
			}
			else
			{
				sparki.moveRight(EDGE_TURNAROUND);
				count = 0;
			}
		}
		else
		{
			sparki.moveRight(EDGE_TURN);
			count++;
		}
	}

	if (edgeRight < NO_FLOOR) // if no surface underneath right sensor
	{
		sparki.RGB(RGB_WHITE);
		
		if (count > COUNT_THRESHOLD)
		{
			if (RANUM > SMALL_RANDOM_ADJUST)
			{
				if (RANUM%2 == 0)// 50/50 if statement
				{
					EDGE_TURNAROUND -= RANUM;
					sparki.moveLeft(EDGE_TURNAROUND);
					count = 0;
				}
				else
				{
					EDGE_TURNAROUND += RANUM;
					sparki.moveLeft(EDGE_TURNAROUND);
					count = 0;
				}
			}
			else
			{
				sparki.moveLeft(EDGE_TURNAROUND);
				count = 0;
			}
		}
		else
		{
			sparki.moveLeft(EDGE_TURN);
			count++;
		}
	}
	
	sparki.moveForward();
}

//----------------------------------------------------------------------------------------------------------
//	void behaviorsGO() this method checks the RANUM and fires a behavior if the params are right
//----------------------------------------------------------------------------------------------------------
void behaviorsGO()
{
	//switch for actual random behaviors
	switch(RANUM)
	{
		case 2:
			randomDirection();
			break;
		case 15:
			investigateObject();
			break;
		case 30:
			lookAbout();
			break;
		case 42:
			baladOfBobLegend();
			break;
		case 65:
			gripperFlex();
			break;
		case 99:
			spinningTheramin();
			break;
		default:
			//take no action
			break;
	}
}

//----------------------------------------------------------------------------------------------------------
//	void randomDirection() stops and picks a random direction
//YELLOW
//----------------------------------------------------------------------------------------------------------
void randomDirection()
{
	RANUM = random(RAN_MIN, RAN_MAX);
	
	sparki.println("randomDirection()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	sparki.RGB(RGB_YELLOW);
	sparki.moveStop();
	busyBeep();
	
	if(RANUM%2 == 0)// 50/50 if statement
	{
		if (RANUM < HALF_RANUM)
		{
			sparki.moveRight(RANUM + SUM_RANUM);
		}
		else
		{
			sparki.moveRight(RANUM);	
		}
		
		fastBusyBeep();
	}
	else
	{
		if (RANUM < HALF_RANUM)
		{
			sparki.moveLeft(RANUM + SUM_RANUM);
		}
		else
		{
			sparki.moveLeft(RANUM);	
		}
		
		fastBusyBeep();
	}
}

//----------------------------------------------------------------------------------------------------------
//	void investigateObject() BEHAVIOR this method stops sparki and has him look around for a nearby object to check out
// 	BLUE
//----------------------------------------------------------------------------------------------------------
void investigateObject()
{  
	sparki.println("investigateObject()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();

	//pick variables
	int tempPing = 0, decision = 0, rightPing = 0, fullRightPing = 0, leftPing = 0, fullLeftPing = 0;
	
	//centering servo before scan
	sparki.servo(SERVO_CENTER);
	sparki.RGB(RGB_BLUE);

	sparki.moveStop();
	delay(TWENTIETH_SECOND_DELAY);    
      
	//get right ping
	sparki.servo(LOOK_RIGHT);
	delay(TENTH_SECOND_DELAY);
	rightPing = sparki.ping();
	
	//get full right ping
	sparki.servo(LOOK_FULL_RIGHT);
	delay(TENTH_SECOND_DELAY);
	fullRightPing = sparki.ping();

	//get left ping
	sparki.servo(LOOK_LEFT);
	delay(FIFTH_SECOND_DELAY);
	leftPing = sparki.ping();
      
	//get full left ping
	sparki.servo(LOOK_FULL_LEFT);
	delay(TENTH_SECOND_DELAY);
	fullLeftPing = sparki.ping();
	
	//re-centering servo after scan
	sparki.servo(SERVO_CENTER);

	//make decisions for 5 different outcomes
	//1 = turn Right
	//2 = full Right
	//3 = turn Left
	//4 = full Left
	//5 = sadness
	
	//make decision
	if (leftPing <= rightPing)
	{
		tempPing = leftPing;
		decision = 3;
	}
	else
	{
		tempPing = rightPing;
		decision = 1;
	}
	if (fullLeftPing < tempPing)
	{
		tempPing = fullLeftPing;
		decision = 4;
	}
	if (fullRightPing <= tempPing)
	{
		tempPing = fullRightPing;
		decision = 2;
	}
	if (rightPing >= CM_TOO_FAR && leftPing >= CM_TOO_FAR && fullRightPing >= CM_TOO_FAR && fullLeftPing >= CM_TOO_FAR)
	{
		decision = 5;
	}
	
	switch(decision)
	{
		case 0:
			//sad
			sadBeep();
			break;
		case 1:
			sparki.moveRight(TURN);
			moveToInvestigate();
			break;
		case 2:
			sparki.moveRight(FULL_TURN);
			moveToInvestigate();
			break;
		case 3:
			sparki.moveLeft(TURN);
			moveToInvestigate();
			break;
		case 4:
			sparki.moveLeft(FULL_TURN);
			moveToInvestigate();
			break;
		case 5:
			//sad
			sadBeep();
			break;
		default:
			//sad
			sadBeep();
			break;  
	}
}

//----------------------------------------------------------------------------------------------------------
//	void moveToInvestigate() BEHAVIOR SUBCLASS this method moves sparki close to an object to check out
//----------------------------------------------------------------------------------------------------------
void moveToInvestigate()
{
	int ping;
	
	RANUM = random(RAN_MIN, RAN_MAX);
	
	sparki.println("edgeDetection()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	do
	{
		ping = sparki.ping();
		sparki.moveForward();
		delay(TWENTIETH_SECOND_DELAY);
	}while (ping > CM_OBJECT_THRESHOLD);
	
	sparki.moveStop();
	happyBeep();
	
	sparki.servo(LOOK_SLIGHT_RIGHT);
	delay(TENTH_SECOND_DELAY);

	sparki.servo(LOOK_SLIGHT_LEFT);
	delay(TENTH_SECOND_DELAY);
	
	sparki.servo(LOOK_SLIGHT_RIGHT);
	delay(TENTH_SECOND_DELAY);

	sparki.servo(LOOK_SLIGHT_LEFT);
	delay(TENTH_SECOND_DELAY);
	sparki.servo(SERVO_CENTER);
	
	if(RANUM%2 == 0)// 50/50 if statement
	{
		sparki.moveRight(SPIN);
		happyBeep();
	}
	else
	{
		sparki.moveLeft(SPIN);
		happyBeep();
	}
	
	if(RANUM%2 == 0)// 50/50 if statement
	{
		sparki.moveLeft(RANUM);
		happyBeep();
	}
	else
	{
		sparki.moveRight(RANUM);
		happyBeep();
	}
	
}

//----------------------------------------------------------------------------------------------------------
//	void lookAbout() BEHAVIOR sparki stops, looks around and chirps
//	ORANGE
//----------------------------------------------------------------------------------------------------------
void lookAbout()
{
	int loopInt = 0;
	
	sparki.println("lookAbout()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	sparki.moveStop();
	
	busyBeep();
	sparki.RGB(RGB_ORANGE);
	
	while (loopInt > LOOK_LEFT_MAX)
	{
		sparki.servo(loopInt);
		delay(FOURTIETH_SECOND_DELAY);
		loopInt -= LOOP_INCREMENT;
	}
	
	while (loopInt < LOOK_RIGHT_MAX)
	{
		sparki.servo(loopInt);
		delay(FOURTIETH_SECOND_DELAY);
		loopInt += LOOP_INCREMENT;
	}
	
	while (loopInt > LOOK_CENTER)
	{
		sparki.servo(loopInt);
		delay(FOURTIETH_SECOND_DELAY);
		loopInt -= LOOP_INCREMENT;
	}
	
	happyBeep();
}

//----------------------------------------------------------------------------------------------------------
//	void baladOfBobLegend() BEHAVIOR stops to sing the baladOfBobLegend
//	INDIGO
//----------------------------------------------------------------------------------------------------------
void baladOfBobLegend()
{
	sparki.println("baladOfBobLegend()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	//magic numbers in this method are left behind because the values are so exclusive as to be un-reusable
	//see Ben Robinson for details on how this bit works; he did all the music magics
	sparki.moveStop();
	sparki.RGB(RGB_INDIGO);
	//sing the balad of bob legend
	// notes in the melody:
	int melody[] = { NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B3, NOTE_B3, NOTE_C4, NOTE_B3, NOTE_G3, NOTE_B3, NOTE_A3, NOTE_G3, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_B3, NOTE_C4,      
                  NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B3, NOTE_B3, NOTE_C4, NOTE_B3, NOTE_G3, NOTE_B3, NOTE_A3, NOTE_G3, NOTE_A3, 0};

	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, 8, 8, 4, 4, 8, 8, 4, 4, 4, 4, 4, 8, 8, 8, 8, 4, 8, 8, 4, 4, 8, 8, 4, 4, 4, 4, 4, 2};
	
	for (int thisNote = 0; thisNote < 29; thisNote++) 
	{
	
	//if statements comprise BOBs scripted "dance" he performs while singing
	if (thisNote == 0)
		sparki.servo(LOOK_LEFT);
	
	if (thisNote == 2)
		sparki.moveRight();
	
	if (thisNote == 4)
		sparki.servo(LOOK_RIGHT);
	
	if (thisNote == 6)
		sparki.moveLeft();
	
	if (thisNote == 8)
		sparki.servo(LOOK_LEFT);
	
	if (thisNote == 10)
		sparki.moveRight();
	
	if (thisNote == 12)
		sparki.servo(LOOK_RIGHT);
	
	if (thisNote == 14)
		sparki.moveLeft();
	
	if (thisNote == 16)
		sparki.servo(LOOK_LEFT);
	
	if (thisNote == 18)
		sparki.moveRight();
	
	if (thisNote == 20)
		sparki.servo(LOOK_RIGHT);
	
	if (thisNote == 22)
		sparki.moveLeft();
	
	if (thisNote == 24)
		sparki.servo(LOOK_LEFT);
	
	if (thisNote == 26)
		sparki.moveRight();
	
	if (thisNote == 28)
		sparki.servo(LOOK_RIGHT);
	
    int noteDuration = 1000/noteDurations[thisNote];
    sparki.beep(melody[thisNote],noteDuration);

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    sparki.noBeep();
  }
  
  sparki.servo(SERVO_CENTER);
}

//----------------------------------------------------------------------------------------------------------
//	void gripperFlex() BEHAVIOR flexes his gripper while moving
// 	WILL CAUSE FLASHING!!!!!  ORANGE
//----------------------------------------------------------------------------------------------------------
void gripperFlex()
{
	sparki.println("gripperFlex()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	int delayInt = DELAYINT_INCREMENT;
	
	sparki.RGB(RGB_ORANGE);
	
	sparki.gripperClose(GRIPPER_MOVE_CM);
	while(delayInt < TWO_SECOND_DELAY)
	{
		pickRoute();
		delayInt += DELAYINT_INCREMENT;
	}
	
	delayInt = DELAYINT_INCREMENT;
	sparki.gripperOpen(GRIPPER_MOVE_CM);
	while(delayInt < TWO_SECOND_DELAY)
	{
		pickRoute();
		delayInt += DELAYINT_INCREMENT;
	}
	
	delayInt = DELAYINT_INCREMENT;
	sparki.gripperClose(GRIPPER_MOVE_CM);
	while(delayInt < TWO_SECOND_DELAY)
	{
		pickRoute();
		delayInt += DELAYINT_INCREMENT;
	}
	
	delayInt = DELAYINT_INCREMENT;
	sparki.gripperOpen(GRIPPER_MOVE_CM);
	while(delayInt < TWO_SECOND_DELAY)
	{
		pickRoute();
		delayInt += DELAYINT_INCREMENT;
	}
}

//----------------------------------------------------------------------------------------------------------
//	void spinningTheramin() runs the spinning theramin for 360deg
// 	ORANGE
//----------------------------------------------------------------------------------------------------------
void spinningTheramin()
{
	sparki.println("spinningTheramin()");
	sparki.print("RANUM: ");
	sparki.println(RANUM);
	sparki.updateLCD();
	
	sparki.servo(SERVO_CENTER);
	sparki.RGB(RGB_ORANGE);
  
	for (int i = 0; i < LOOP_TIMED_SPIN; i++)
	{
		int note = sparki.ping();
		sparki.moveRight();
		delay(HUNDREDTH_SECOND_DELAY);
	  
		if (note != CM_ERROR)
		{
			sparki.beep(note * BEEP_MULIPLIER);
		}

		if (note > BEEP_MAX_RANGE)
		{
			sparki.noBeep();
		}

		delay(HUNDREDTH_SECOND_DELAY); 
	}
}

//----------------------------------------------------------------------------------------------------------
// BOBs beeps
//----------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------
//	void happyBeep() sparkis happy beep
//----------------------------------------------------------------------------------------------------------
void happyBeep()
{
	sparki.beep(NOTE_D6, 125);
	delay(125);
	sparki.beep(NOTE_A7, 300);
	delay(300);
	sparki.beep(NOTE_D7, 125);
	delay(125);
	sparki.beep(NOTE_A7, 200);
	delay(200);
}

//----------------------------------------------------------------------------------------------------------
//	void sadBeep() sparkis sad beep
//----------------------------------------------------------------------------------------------------------
void sadBeep()
{
	sparki.beep(NOTE_A6, 400);
	delay(400);
	sparki.beep(NOTE_G4, 400);
	delay(400);
	sparki.beep(NOTE_D3, 400);
	delay(400);
}

//----------------------------------------------------------------------------------------------------------
//	void busyBeep() sparkis doing things beep
//----------------------------------------------------------------------------------------------------------
void busyBeep()
{
	sparki.beep(NOTE_C3, 200);
	delay(300);
	sparki.beep(NOTE_C3, 200);
	delay(300);
	sparki.beep(NOTE_C3, 200);
	delay(300);
	sparki.beep(NOTE_C3, 200);
	delay(300);
	sparki.beep(NOTE_G3, 200);
	delay(300);
	sparki.beep(NOTE_G3, 200);
	delay(300);
	sparki.beep(NOTE_G3, 200);
	delay(300);
	sparki.beep(NOTE_G3, 200);
	delay(300);
}

//----------------------------------------------------------------------------------------------------------
//	void fastBusyBeep() sparkis doing things beep
//----------------------------------------------------------------------------------------------------------
void fastBusyBeep()
{
	sparki.beep(NOTE_C3, 100);
	delay(150);
	sparki.beep(NOTE_C3, 100);
	delay(150);
	sparki.beep(NOTE_C3, 100);
	delay(150);
	sparki.beep(NOTE_C3, 100);
	delay(150);
	sparki.beep(NOTE_G3, 100);
	delay(150);
	sparki.beep(NOTE_G3, 100);
	delay(150);
	sparki.beep(NOTE_G3, 100);
	delay(150);
	sparki.beep(NOTE_G3, 100);
	delay(150);
}