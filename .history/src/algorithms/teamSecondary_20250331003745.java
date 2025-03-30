/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/teamSecondary.java 2025-03-27 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult.Types;
import characteristics.Parameters.Direction;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;
import java.util.HashMap;


public class teamSecondary extends Brain {

  //---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.001;
  private static final double ANGLEPRECISIONBIS = 0.03;


  private static final double MAIN = 0xAAAAFFFF;
  private static final double SECONDARY = 0xFFFFAAAA;

  private static final int ROCKY = 0x1EBDDB;
  private static final int MARIO = 0x5ECD;
  private static final int ALPHA = 0x1EADDA;
  private static final int BETA = 0x5EC0;
  private static final int GAMMA = 0x333;
  private static final int TEAM = 0xBADDAD;

  private static final int FIRE = 0xB52;
  private static final int POSITION = 0x7E57;
  private static final int OVER = 0xC00010FF;

  private static final int TURNNORTHTASK = 1;
  private static final int TURNSOUTHTASK = 2;
  private static final int TURNEASTTASK = 3;
  private static final int TURNWESTTASK = 4;
  private static final int MOVETASK = 5;
  private static final int FIRSTMOVETASK = 6;
  private static final int FLEE = 7;
  private static final int TURNLEFTTASK = 8;
  private static final int MOVEBACKTASK = 9;
  private static final int TURNRIGHTTASK = 10;
  private static final int FIRSTTURNNORTHTASK = 11;
  private static final int FIRSTTURNSOUTHTASK = 22;
  private static final int SINK = 0xBADC0DE1;

  private boolean myTeam;
  //---VARIABLES---//
  private int state;
  private double myX,myY;
  private boolean isMoving;
  private int whoAmI;
  private HashMap<Integer, ArrayList<Double>> allies;
  private ArrayList<IRadarResult> ennemies;
  private double endTaskDirection;
  private int stepNumber, stepNumberMoveBack;
  private boolean isMovingBack;
  private boolean allyOnLeft;
  private boolean allyOnRight;

  //---CONSTRUCTORS---//
  public teamSecondary() {
    super();
    allies = new HashMap<Integer, ArrayList<Double>>(5);
    ArrayList<Double> temp = new ArrayList<Double>(2);
    temp.add(0.);
    temp.add(0.);
    allies.put(ALPHA, temp);
    allies.put(BETA, temp);
    allies.put(GAMMA, temp);
    allies.put(ROCKY, temp);
    allies.put(MARIO, temp);
    ennemies = new ArrayList<IRadarResult>();
  }

  //---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    myTeam = determineTeam();
;
    //ODOMETRY CODE
    whoAmI = ROCKY;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.NORTH)) whoAmI=MARIO;
    if (myTeam) {
    		if (whoAmI == ROCKY){
    	      myX=Parameters.teamASecondaryBot1InitX;
    	      myY=Parameters.teamASecondaryBot1InitY;
    	      state=FIRSTTURNNORTHTASK;
    	    } else {
    	      myX=Parameters.teamASecondaryBot2InitX;
    	      myY=Parameters.teamASecondaryBot2InitY;
    	      state=FIRSTTURNSOUTHTASK;
    	    }

    } else {
	    if (whoAmI == ROCKY){
	      myX=Parameters.teamBSecondaryBot1InitX;
	      myY=Parameters.teamBSecondaryBot1InitY;
	      state=FIRSTTURNNORTHTASK;
	    } else {
	      myX=Parameters.teamBSecondaryBot2InitX;
	      myY=Parameters.teamBSecondaryBot2InitY;
	      state=FIRSTTURNSOUTHTASK;
	    }
    }
    
    //INIT
    isMoving=false;
    stepNumber=0;
    stepNumberMoveBack=0;
    isMovingBack=false;
	allyOnLeft = false;
	allyOnRight = false;
  }
  public void step() {
	stepNumber++;


	// Check if there are allies in left or right directions
	for (IRadarResult o: detectRadar()) {
		if (o.getObjectType() == Types.TeamMainBot || o.getObjectType() == Types.TeamSecondaryBot) {

			double relativeDirection = normalizeRadian(o.getObjectDirection() - getHeading());
			
			// Left side detection - between 90° and 270° relative to heading
			if (relativeDirection > 4 && relativeDirection < 5) {
				allyOnLeft = true;

			}
			
			// Right side detection - between 270° and 360° OR between 0° and 90° relative to heading
			if (relativeDirection < 2 && relativeDirection > 1) {
				allyOnRight = true;
			}
		}
	}

	if (getHealth()==0) state=SINK;
	
    //ODOMETRY CODE
    if (isMoving){
      myX+=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
      myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
      realCoords();
      isMoving=false;
    }
    if (isMovingBack) {
		myX-=Parameters.teamAMainBotSpeed*Math.cos(myGetHeading());
		myY-=Parameters.teamAMainBotSpeed*Math.sin(myGetHeading());
		realCoords();
		isMovingBack=false;
    	}
    //DEBUG MESSAGE
    if (whoAmI == ROCKY) sendLogMessage("#ROCKY *thinks* he is rolling at position ("+(int)myX+", "+(int)myY+")." + "#state:"+state);
    else sendLogMessage("#MARIO *thinks* he is rolling at position ("+(int)myX+", "+(int)myY+")."+ "#state:"+state);

    //RADAR DETECTION
    for (IRadarResult o: detectRadar()){
      if (o.getObjectType()== Types.OpponentMainBot || o.getObjectType()== Types.OpponentSecondaryBot) {
        double enemyX=myX+o.getObjectDistance()*Math.cos(o.getObjectDirection());
        double enemyY=myY+o.getObjectDistance()*Math.sin(o.getObjectDirection());
        broadcast(whoAmI+":"+TEAM+":"+FIRE+":"+(o.getObjectType()== Types.OpponentMainBot?MAIN:SECONDARY)+":"+enemyX+":"+enemyY+":"+OVER);
        ennemies.add(o);
      }
    }
    //lastSeenEnnemies = new ArrayList<>(ennemies);
    //COMMUNICATION
    broadcast(whoAmI+":"+TEAM+":"+POSITION+":"+myX+":"+myY+":"+myGetHeading()+":"+OVER);

    ennemies.clear();
    for (IRadarResult o: detectRadar()){
    		//détection des ennemies
	  if ((o.getObjectType()== Types.OpponentMainBot &&
	      o.getObjectDistance() <= Parameters.teamBMainBotFrontalDetectionRange + 100) ||
	      (o.getObjectType()== Types.OpponentSecondaryBot &&
	      o.getObjectDistance() <= Parameters.teamASecondaryBotFrontalDetectionRange - 150)) {
	    ennemies.add(o);
	    if (state==MOVETASK)
	      state=FLEE;
	  }
	  //détection de collision avec un membre de mon équipe ou une épave
	  	if (o.getObjectDistance() < 120 && o.getObjectType() != Types.BULLET) {
			if (state==MOVETASK) {
				state=MOVEBACKTASK;
				stepNumberMoveBack = stepNumber;
			}
		}
    }
    
    //AUTOMATON
    
    /* when bot is sticked to wall */
    if (myX<=Parameters.teamASecondaryBotRadius) {
		if (isHeading(Parameters.EAST)) {
		state=MOVETASK;
		return;
	}
	state=TURNEASTTASK;
	return;
	}
	if (myX>=(3000-Parameters.teamASecondaryBotRadius)) {
		if (isHeading(Parameters.WEST)) {
			state=MOVETASK;
			return;
		}
		state=TURNWESTTASK;
		return;
	}
	if (myY<=Parameters.teamASecondaryBotRadius) {
		if (isHeading(Parameters.SOUTH)) {
			state=MOVETASK;
			return;
		}
		state=TURNSOUTHTASK;
		return;
	}
	if (myY>=(2000-Parameters.teamASecondaryBotRadius)) {
		state=TURNNORTHTASK;
		if (isHeading(Parameters.NORTH)) {
			state=MOVETASK;
			return;
		}
		return;
	}
	
	//FIRST MOVE 
	if (state==FIRSTMOVETASK) {
		myMove(); //And what to do when blind blocked?
		if (whoAmI == MARIO) {
			if ((myTeam && myY > 1325) || (!myTeam && myY > 1325)) { 
				if (myTeam) {
					state=TURNEASTTASK;	    			  
				} else {
					state = TURNWESTTASK;
				}
				return;
			}
		} else { // ROCKY
			if ((myTeam && myY < 650) || (!myTeam && myY < 650)) {
				if (myTeam) {
					state=TURNEASTTASK;	    			  
				} else {
					state = TURNWESTTASK;
				}
				return;
			}
		}	
	}
	
	if (state==FIRSTTURNNORTHTASK && !(isHeading(myTeam ? 5*Math.PI/6 : Math.PI/6))) {
		// Turn toward southeast (team A) or northeast (team B)
		double targetAngle = myTeam ? 5*Math.PI/6 : Math.PI/6;
		if ((myTeam && (myGetHeading() < Math.PI/12 || myGetHeading() > 11*Math.PI/6)) || 
			(!myTeam && myGetHeading() > 7*Math.PI/6)) {
			stepTurn(Direction.LEFT);
		}
		else {
			stepTurn(Direction.RIGHT);
		}
		return;
	}
	if (state==FIRSTTURNNORTHTASK && isHeading(myTeam ? 5*Math.PI/6 : Math.PI/6)) {
		state=FIRSTMOVETASK;
		myMove();
		return;
	}
	if (state==FIRSTTURNSOUTHTASK && !(isHeading(myTeam ? Math.PI/6 : 5*Math.PI/6))) {
		// Turn toward northeast (team A) or southeast (team B)
		double targetAngle = myTeam ? Math.PI/6 : 5*Math.PI/6;
		if ((myTeam && myGetHeading() < 5*Math.PI/6 && myGetHeading() > Math.PI/6) || 
			(!myTeam && (myGetHeading() < Math.PI/6 || myGetHeading() > 5*Math.PI/6))) {
			stepTurn(Direction.LEFT);
		}
		else {
			stepTurn(Direction.RIGHT);
		}
		return;
	}
	if (state==FIRSTTURNSOUTHTASK && isHeading(myTeam ? Math.PI/6 : 5*Math.PI/6)) {
		state=FIRSTMOVETASK;
		myMove();
		return;
	}

    /* 4 directions turning */
    if (state==TURNNORTHTASK && !(isHeading(Parameters.NORTH))) {
      if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
        stepTurn(Direction.LEFT);
      }
      else {
        stepTurn(Direction.RIGHT);
      }
      return;
    }
    if (state==TURNNORTHTASK && isHeading(Parameters.NORTH)) {
      state=MOVETASK;
      myMove();
      return;
    }
    if (state==TURNSOUTHTASK && !(isHeading(Parameters.SOUTH))) {
      if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
        stepTurn(Direction.RIGHT);
      }
      else {
        stepTurn(Direction.LEFT);
      }
      return;
    }
    if (state==TURNSOUTHTASK && isHeading(Parameters.SOUTH)) {
    		state=MOVETASK;
    		myMove();
    		return;
    }
    if (state==TURNEASTTASK && !(isHeading(Parameters.EAST))) {
      if (myGetHeading() < Math.PI && myGetHeading() > 0) {
        stepTurn(Direction.LEFT);
      }
      else {
        stepTurn(Direction.RIGHT);
      }
      return;
    }
    if (state==TURNEASTTASK && isHeading(Parameters.EAST)) {
      state=MOVETASK;
      myMove();
      return;
    }
    if (state==TURNWESTTASK && !(isHeading(Parameters.WEST))) {
      if (myGetHeading() < Math.PI && myGetHeading() > 0) {
        stepTurn(Direction.RIGHT);
      }
      else {
        stepTurn(Direction.LEFT);
      }
      return;
    }
    if (state==TURNWESTTASK && isHeading(Parameters.WEST)) {
      state=MOVETASK;
      myMove();
      return;
    }
    
    
    if (state==MOVETASK) {
    		if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {	
    			if (whoAmI == MARIO) {
    				if ((myX>2800 && myY>1800) || (myX > 2800 && myY<200) || (myX<200 && myY<200) || (myX<200 && myY>1800)) {
    					state=TURNLEFTTASK;
    	    		  		endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    	    		  		stepTurn(Direction.LEFT);
    	    		  		return;
    				} else if (myX>2800 || myX<200) {
    					if (isHeading(Parameters.EAST) || isHeading(Parameters.WEST)){
    						state=TURNLEFTTASK;
    			  			endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    			  			stepTurn(Direction.LEFT);
    			  			return;
    					} else {
    						myMove();
    						return;
    					}
    				} else if (myY>1800 || myY<200){
    					if (isHeading(Parameters.NORTH) || isHeading(Parameters.SOUTH)){
    						state=TURNLEFTTASK;
    			  			endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    			  			stepTurn(Direction.LEFT);
    			  			return;
    					} else {
    						myMove();
    						return;
    					}
    				} else {
    					myMove();
    					return;
    				}
    				
    			} else { //rocky fait un tour normal
	    		  state=TURNLEFTTASK;
	    		  endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
	    		  stepTurn(Direction.LEFT);
	    		  return;
    			}
    		} 
    		myMove();
    		return;
	    
    }
    
    if (state==TURNRIGHTTASK) {
		if (isHeading(endTaskDirection)) {
			state=MOVETASK;
			myMove();
		} else {
			stepTurn(Direction.RIGHT);
		}
		return;
	}
    
    if (state==TURNLEFTTASK) {
		if (isHeading(endTaskDirection)) {
			state=MOVETASK;
			myMove();
		} else {
			stepTurn(Direction.LEFT);
		}
		return;
	}
    
    if (state==MOVEBACKTASK) {
    		if (stepNumber < stepNumberMoveBack + 25 && 
    		    myX > Parameters.teamASecondaryBotRadius*2 && 
    		    myX < (3000-Parameters.teamASecondaryBotRadius*2) && 
    		    myY > Parameters.teamASecondaryBotRadius*2 && 
    		    myY < (2000-Parameters.teamASecondaryBotRadius*2)) {
    			myMoveBack();
    			return;
    		} else {


    			// Choose turn direction based on ally positions
    			if (allyOnLeft && !allyOnRight) {
    			    // If ally on left, turn right
    			    state = TURNRIGHTTASK;
    			    endTaskDirection = getHeading() + Parameters.RIGHTTURNFULLANGLE;
    			    stepTurn(Direction.RIGHT);
    			} else if (!allyOnLeft && allyOnRight) {
    			    // If ally on right, turn left
    			    state = TURNLEFTTASK;
    			    endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    			    stepTurn(Direction.LEFT);
    			} else {
    			    // If allies on both sides or no allies, random choice as before
    			    if (Math.random() < 0.5) {
    			        state = TURNLEFTTASK;
    			        endTaskDirection = getHeading() + Parameters.LEFTTURNFULLANGLE;
    			        stepTurn(Direction.LEFT);
    			    } else {
    			        state = TURNRIGHTTASK;
    			        endTaskDirection = getHeading() + Parameters.RIGHTTURNFULLANGLE;
    			        stepTurn(Direction.RIGHT);
    			    }
    			}

				allyOnLeft = false;
				allyOnRight = false
    			return;
    		}
    }
    
  

    if (state==FLEE) {
    		if ((myX>2900 || myX<100) && (myY>1900 || myX<100)){
			state=TURNRIGHTTASK;
		  	endTaskDirection = getHeading() + Parameters.RIGHTTURNFULLANGLE;
		  	stepTurn(Direction.RIGHT);
		  	return;
		} else if (myX>2900 || myX<100) {
			if (isHeading(Parameters.EAST) || isHeading(Parameters.WEST)){
				state=TURNRIGHTTASK;
	  			endTaskDirection = getHeading() + Parameters.RIGHTTURNFULLANGLE;
	  			stepTurn(Direction.RIGHT);
	  			return;
			} else {
				moveBack();
	    			myX-=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
	    			myY-=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
	    			realCoords();
	    			if (ennemies.isEmpty()) {
	    				state=MOVETASK;
	    			}
	    			return;
			}
		} else if (myY>1900 || myY<100){
			if (isHeading(Parameters.NORTH) || isHeading(Parameters.SOUTH)){
				state=TURNRIGHTTASK;
	  			endTaskDirection = getHeading() + Parameters.RIGHTTURNFULLANGLE;
	  			stepTurn(Direction.RIGHT);
	  			return;
			} else {
				moveBack();
    				myX-=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
    				myY-=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
    				realCoords();
    				if (ennemies.isEmpty()) {
    					state=MOVETASK;
    				}
    				return;
			}
		} else {
			moveBack();
			myX-=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
			myY-=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
			realCoords();
			if (ennemies.isEmpty()) {
				state=MOVETASK;
			}
			return;
		}
    		
    }

    if (state==SINK) {
      return;
    }
    if (true) {
      return;
    }
  }
  private void myMove(){
    isMoving=true;
    move();
  }
  private void myMoveBack() {
	 isMovingBack=true;
	 moveBack();
  }
  private double myGetHeading(){
    return normalizeRadian(getHeading());
  }
  private double normalizeRadian(double angle){
    double result = angle;
    while(result<0) result+=2*Math.PI;
    while(result>=2*Math.PI) result-=2*Math.PI;
    return result;
  }

  private boolean isSameDirection(double dir1, double dir2){
    return Math.abs(dir1-dir2)<ANGLEPRECISION;
  }
  private boolean isHeading(double dir){
    return Math.abs(Math.sin(normalizeRadian(getHeading())-dir))<ANGLEPRECISIONBIS;
  }
  private void realCoords() {
    myX = myX < 0 ? 0 : myX;
    myX = myX > 3000 ? 3000 : myX;
    myY = myY < 0 ? 0 : myY;
    myY = myY > 2000 ? 2000 : myY;
  }

  private boolean determineTeam() {
    if (isSameDirection(getHeading(), Parameters.EAST)) {
      return true;
    }
    return false;
  }
}