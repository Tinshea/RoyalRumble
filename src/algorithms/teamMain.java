/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/teamMain.java 2025-03-25 .
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IRadarResult.Types;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Comparator;
import java.util.Random;

public class teamMain extends Brain {

  // ---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.001;
  private static final double ANGLEPRECISIONBIS = 0.01;
  private static final double ANGLE_PRECISION = 0.03;
  private static final double DISTANCEPRECISION = 10.;
  private static final double SAMELINEPRECISION = 125.;
  private static final double TRIANGLE_SIDE = 250;

  // Adding threat threshold and priority constants
  private static final double HIGH_THREAT_DISTANCE = 300.0;
  private static final double MEDIUM_THREAT_DISTANCE = 600.0;
  private static final int HIGH_PRIORITY = 3;
  private static final int MEDIUM_PRIORITY = 2;
  private static final int LOW_PRIORITY = 1;

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

  // ---TASK CONSTANTS---//
  private static final int MOVETASK = 1;
  private static final int STANDINGFIRINGTASK = 2;
  private static final int BACKWARDFIRINGTASK = 3;
  private static final int MOVEBACKTASK = 5;
  private static final int TURNLEFTTASK = 6;
  private static final int TURNRIGHTTASK = 7;
  private static final int TRIANGLE_FORMATION = 8;
  private static final int HUNTINGTASK = 9;
  private static final int TURNNORTHTASK = 10;
  private static final int TURNSOUTHTASK = 11;
  private static final int TURNEASTTASK = 12;
  private static final int TURNWESTTASK = 13;
  private static final int FINAL_ORIENTATION = 14;

  // ---PARAMETERS---//
  private static final int FIRE_COOLDOWN = 3;
  private static final int SINK = 0xBADC0DE1;

  // ---ROBOT STATE---//
  private int currentState; // Current robot state/task
  private int firingSequenceStep; // Step counter for firing sequence
  private double positionX, positionY; // Current position
  private boolean isMovingForward; // Flag for forward movement
  private boolean isMovingBackward; // Flag for backward movement
  private int robotId; // Robot identifier
  private int cooldownCounter; // Weapon cooldown counter

  // ---TARGETING---//
  private double targetX, targetY; // Current enemy target coordinates
  private boolean firingEnabled; // Whether robot should fire
  private Random randomGenerator; // Random number generator
  private HashMap<Integer, ArrayList<Double>> teammates; // Teammate positions and headings
  private ArrayList<ArrayList<Double>> enemyTargets; // Detected enemy positions

  // ---TIMING---//
  private int currentStep; // Current simulation step
  private int backMoveStartStep; // When backward movement started

  // ---MOVEMENT---//
  private double targetRotation; // Desired heading after rotation
  private String huntingStrategy; // Strategy for approaching targets
  private boolean needNewAimAngle; // Whether to recalculate firing angle
  private int counter;
  private boolean myTeam;
  private double lastTargetX, lastTargetY; // Remember last target position
  private int targetLockCount; // Track how long we've been targeting the same enemy
  private double formationX, formationY;

  // ---CONSTRUCTORS---//
  public teamMain() {
    super();
    teammates = new HashMap<Integer, ArrayList<Double>>();
    ArrayList<Double> temp = new ArrayList<Double>(3);
    temp.add(0.);
    temp.add(0.);
    temp.add(0.);
    teammates.put(ALPHA, temp);
    teammates.put(BETA, temp);
    teammates.put(GAMMA, temp);
    teammates.put(ROCKY, temp);
    teammates.put(MARIO, temp);
    enemyTargets = new ArrayList<ArrayList<Double>>(5);
    randomGenerator = new Random();
    lastTargetX = 0;
    lastTargetY = 0;
    targetLockCount = 0;
  }

  // ---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    myTeam = determineTeam();
    // ODOMETRY CODE
    robotId = GAMMA;
    for (IRadarResult o : detectRadar())
      if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
        robotId = ALPHA;
    for (IRadarResult o : detectRadar())
      if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH) && robotId != GAMMA)
        robotId = BETA;

    if (myTeam) {
      // KD RUNNERS
      if (robotId == GAMMA) {
        positionX = Parameters.teamAMainBot1InitX;
        positionY = Parameters.teamAMainBot1InitY;
      } else {
        positionX = Parameters.teamAMainBot2InitX;
        positionY = Parameters.teamAMainBot2InitY;
      }
      if (robotId == ALPHA) {
        positionX = Parameters.teamAMainBot3InitX;
        positionY = Parameters.teamAMainBot3InitY;
      }
    } else {
      // FANTOM DANGER
      if (robotId == GAMMA) {
        positionX = Parameters.teamBMainBot1InitX;
        positionY = Parameters.teamBMainBot1InitY;
      } else {
        positionX = Parameters.teamBMainBot2InitX;
        positionY = Parameters.teamBMainBot2InitY;
      }
      if (robotId == ALPHA) {
        positionX = Parameters.teamBMainBot3InitX;
        positionY = Parameters.teamBMainBot3InitY;
      }
    }

    double centerX = (myTeam) ? Parameters.teamAMainBot2InitX + 100 : Parameters.teamBMainBot2InitX - 100;
    double centerY = (myTeam) ? Parameters.teamAMainBot2InitY : Parameters.teamBMainBot2InitY;
    double[] formationPosition = calculateFormationPosition(centerX, centerY, robotId);
    formationX = formationPosition[0];
    formationY = formationPosition[1];

    currentState = TRIANGLE_FORMATION;
    isMovingForward = false;
    firingEnabled = false;
    cooldownCounter = 0;
    targetX = 0;
    targetY = 0;
    currentStep = 0;
    backMoveStartStep = 0;
    needNewAimAngle = false;
    isMovingBackward = false;
    counter = 0;
    targetLockCount = 0;
  }

  public void step() {
    currentStep++;
    cooldownCounter++;
    if (cooldownCounter >= FIRE_COOLDOWN)
      cooldownCounter = 0;

    if (counter > 460) {
      counter = 0;
    }

    if (getHealth() == 0)
      currentState = SINK;

    // ODOMETRY CODE
    if (isMovingForward) {
      positionX += Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
      positionY += Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
      realCoords();
      isMovingForward = false;
    }

    if (isMovingBackward) {
      positionX -= Parameters.teamAMainBotSpeed * Math.cos(myGetHeading());
      positionY -= Parameters.teamAMainBotSpeed * Math.sin(myGetHeading());
      realCoords();
      isMovingBackward = false;
    }
    // DEBUG MESSAGE
    boolean debug = true;
    if (debug && robotId == ALPHA && currentState != SINK) {
      sendLogMessage("#ALPHA *thinks* (x,y)= (" + (int) positionX + ", " + (int) positionY + ") theta= "
          + (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + currentState);
    }
    if (debug && robotId == BETA && currentState != SINK) {
      sendLogMessage("#BETA *thinks* (x,y)= (" + (int) positionX + ", " + (int) positionY + ") theta= "
          + (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + currentState);
    }
    if (debug && robotId == GAMMA && currentState != SINK) {
      sendLogMessage("#GAMMA *thinks* (x,y)= (" + (int) positionX + ", " + (int) positionY + ") theta= "
          + (int) (myGetHeading() * 180 / (double) Math.PI) + "°. #State= " + currentState);
    }
    if (debug && firingEnabled) {
      counter++;
      sendLogMessage("Firing enemy!!");
    }
    if (currentState == TRIANGLE_FORMATION) {
      double distToTarget = Math.hypot(positionX - formationX, positionY - formationY);
      if (distToTarget < 70) {
        currentState = FINAL_ORIENTATION;
      } else {
        moveToCoordinates(formationX, formationY);
        return;
      }

    }

    if (currentState == FINAL_ORIENTATION) {
      double targetDirection = (myTeam) ? Parameters.EAST : Parameters.WEST;
      if (isSameDirection(getHeading(), targetDirection)) {
        currentState = MOVETASK;
      } else {
        double angleDiff = normalizeAngleDifference(targetDirection - getHeading());
        stepTurn(angleDiff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      }
    }

    // COMMUNICATION
    enemyTargets.clear(); // prepare for the next targets set
    ArrayList<String> messages = fetchAllMessages();
    for (String m : messages)
      if (Integer.parseInt(m.split(":")[1]) == robotId || Integer.parseInt(m.split(":")[1]) == TEAM)
        process(m);

    broadcast(
        robotId + ":" + TEAM + ":" + POSITION + ":" + positionX + ":" + positionY + ":" + getHeading() + ":" + OVER);

    // RADAR DETECTION
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == Types.OpponentMainBot || o.getObjectType() == Types.OpponentSecondaryBot) {
        double enemyX = positionX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = positionY + o.getObjectDistance() * Math.sin(o.getObjectDirection());

        // Add threat level assessment
        int threatLevel = assessThreat(o);
        broadcast(
            robotId + ":" + TEAM + ":" + FIRE + ":" + (o.getObjectType() == Types.OpponentMainBot ? MAIN : SECONDARY)
                + ":" + enemyX + ":" + enemyY + ":" + threatLevel + ":" + OVER);
      }
      if ((o.getObjectDistance() < 120 && o.getObjectType() != Types.BULLET)
          || detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
        if (currentState == MOVETASK) {
          currentState = MOVEBACKTASK;
          backMoveStartStep = currentStep;
          return;
        }
      }
    }
    if (firingEnabled && !needNewAimAngle) {
      setTarget();
    }

    // AUTOMATON

    if (positionX <= Parameters.teamAMainBotRadius) {
      if (isHeading(Parameters.EAST)) {
        currentState = MOVETASK;
        return;
      }
      currentState = TURNEASTTASK;
      return;
    }
    if (positionX >= (3000 - Parameters.teamAMainBotRadius)) {
      if (isHeading(Parameters.WEST)) {
        currentState = MOVETASK;
        return;
      }
      currentState = TURNWESTTASK;
      return;
    }
    if (positionY <= Parameters.teamAMainBotRadius) {
      if (isHeading(Parameters.SOUTH)) {
        currentState = MOVETASK;
        return;
      }
      currentState = TURNSOUTHTASK;
      return;
    }
    if (positionY >= (2000 - Parameters.teamAMainBotRadius)) {
      currentState = TURNNORTHTASK;
      if (isHeading(Parameters.NORTH)) {
        currentState = MOVETASK;
        return;
      }
      return;
    }

    if (currentState == TRIANGLE_FORMATION) {
      if (currentStep > 100 && canFireLatency()) {
        if (firingEnabled) {
          if (canIShot(targetX, targetY)) {
            firePosition(targetX, targetY);
            return;
          }
        }
        fire(myGetHeading());
        return;
      }
      myMove();
    }

    if (firingEnabled && canFireLatency()) {
      if (canIShot(targetX, targetY)) {
        firePosition(targetX, targetY);
        return;
      }
    }

    // Improved hunting behavior
    if (!firingEnabled && currentStep > 6000 && enemyTargets.size() > 0 && currentState != HUNTINGTASK) {
      ArrayList<ArrayList<Double>> sortedTargets = prioritizeTargets(enemyTargets);

      if (!sortedTargets.isEmpty()) {
        double tx = sortedTargets.get(0).get(1);
        double ty = sortedTargets.get(0).get(2);
        double distX = Math.abs(tx - positionX);
        double distY = Math.abs(ty - positionY);

        // Enhanced decision making for hunting
        if (distance(positionX, positionY, tx, ty) < 600) {
          currentState = HUNTINGTASK;
          if (distX > distY || distY < 200) {
            huntingStrategy = "x";
          } else {
            huntingStrategy = "y";
          }
          return;
        }
      }
    }

    if (currentState == HUNTINGTASK && !firingEnabled) {
      if (enemyTargets.isEmpty()) {
        currentState = MOVETASK;
        huntingStrategy = "";
        return;
      }

      if (huntingStrategy.equals("x")) {
        double tx = enemyTargets.get(0).get(1);
        double distX = Math.abs(tx - positionX);
        if (distX < 200) {
          huntingStrategy = "";
          currentState = MOVETASK;
          return;
        } else {
          if (tx < positionX) { // target à gauche
            if (isHeading(Parameters.WEST)) {
              myMove();
              return;
            } else {
              if (myGetHeading() < Math.PI && myGetHeading() > 0) {
                stepTurn(Parameters.Direction.RIGHT);
              } else {
                stepTurn(Parameters.Direction.LEFT);
              }
              return;
            }
          } else { // target à gauche
            if (isHeading(Parameters.EAST)) {
              myMove();
              return;
            } else {
              if (myGetHeading() < Math.PI && myGetHeading() > 0) {
                stepTurn(Parameters.Direction.RIGHT);
              } else {
                stepTurn(Parameters.Direction.LEFT);
              }
              return;
            }
          }
        }
      } else {
        double ty = enemyTargets.get(0).get(2);
        double distY = Math.abs(ty - positionY);
        if (distY < 200) {
          huntingStrategy = "";
          currentState = MOVETASK;
          return;
        } else {
          if (ty < positionY) { // target en haut
            if (isHeading(Parameters.NORTH)) {
              myMove();
              return;
            } else {
              if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
                stepTurn(Parameters.Direction.LEFT);
              } else {
                stepTurn(Parameters.Direction.RIGHT);
              }
              return;
            }
          } else { // target en bas
            if (isHeading(Parameters.SOUTH)) {
              myMove();
              return;
            } else {
              if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
                stepTurn(Parameters.Direction.LEFT);
              } else {
                stepTurn(Parameters.Direction.RIGHT);
              }
              return;
            }
          }
        }
      }
    }

    if (currentState == HUNTINGTASK && firingEnabled) {
      currentState = MOVETASK;
      myMove();
      huntingStrategy = "";
      return;
    }

    if (currentState == MOVETASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
      // Check radar for enemies and fire at them directly
      if (canFireLatency()) {
      for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == Types.OpponentMainBot || o.getObjectType() == Types.OpponentSecondaryBot) {
  
        double enemyX = positionX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = positionY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        if (canIShot(enemyX, enemyY)) {
          firePosition(enemyX, enemyY);
          return;
        }
        }
      }
    
        double angle = randomGenerator.nextDouble() * Math.PI / 6 - Math.PI / 12;
        double x = positionX + Parameters.bulletRange * Math.cos(myGetHeading() + angle);
        double y = positionY + Parameters.bulletRange * Math.sin(myGetHeading() + angle);
        if (canIShot(x, y)) {
        firePosition(x, y);
        return;
      }
      }

      myMove();
      return;
    }

    if (currentState == MOVETASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
      if ((positionX > 2915 && positionY > 1915) || (positionX > 2915 && positionY < 85)
          || (positionX < 85 && positionY < 85) || (positionX < 85 && positionY > 1915)) {
        currentState = TURNLEFTTASK;
        targetRotation = getHeading() + Parameters.LEFTTURNFULLANGLE;
        stepTurn(Parameters.Direction.LEFT);
        return;
      } else if (positionX > 2915 || positionX < 85) {
        if (isHeading(Parameters.EAST) || isHeading(Parameters.WEST)) {
          currentState = TURNLEFTTASK;
          targetRotation = getHeading() + Parameters.LEFTTURNFULLANGLE;
          stepTurn(Parameters.Direction.LEFT);
          return;
        } else {
          myMove();
          return;
        }
      } else if (positionY > 1915 || positionY < 85) {
        if (isHeading(Parameters.NORTH) || isHeading(Parameters.SOUTH)) {
          currentState = TURNLEFTTASK;
          targetRotation = getHeading() + Parameters.LEFTTURNFULLANGLE;
          stepTurn(Parameters.Direction.LEFT);
          return;
        } else {
          myMove();
          return;
        }
      } else {
        myMove();
        return;
      }

    }

    if (currentState == STANDINGFIRINGTASK) {
      currentState = MOVETASK;
      if ((++firingSequenceStep % 2) == 0 && counter < 415) {
        moveBack();
        positionX -= Parameters.teamAMainBotSpeed * Math.cos(getHeading());
        positionY -= Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        realCoords();
      } else {
        myMove();
      }
      return;
    }
    if (currentState == BACKWARDFIRINGTASK) {
      currentState = MOVETASK;
      if ((++firingSequenceStep % 1) == 0) {
        moveBack();
        positionX -= Parameters.teamAMainBotSpeed * Math.cos(getHeading());
        positionY -= Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        realCoords();
      } else {
        myMove();
      }
      return;
    }

    if (currentState == MOVEBACKTASK) {
      if (currentStep < backMoveStartStep + 25) {
        myMoveBack();
        return;
      } else {
        if (Math.random() < 0.5) {
          currentState = TURNLEFTTASK;
          targetRotation = getHeading() + Parameters.LEFTTURNFULLANGLE;
          stepTurn(Parameters.Direction.LEFT);
        } else {
          currentState = TURNRIGHTTASK;
          targetRotation = getHeading() + Parameters.RIGHTTURNFULLANGLE;
          stepTurn(Parameters.Direction.RIGHT);
        }
        return;
      }
    }

    if (currentState == TURNRIGHTTASK) {
      if (isHeading(targetRotation)) {
        currentState = MOVETASK;
        myMove();
      } else {
        stepTurn(Parameters.Direction.RIGHT);
      }
      return;
    }

    if (currentState == TURNLEFTTASK) {
      if (isHeading(targetRotation)) {
        currentState = MOVETASK;
        myMove();
      } else {
        stepTurn(Parameters.Direction.LEFT);
      }
      return;
    }

    /* 4 directions turning */
    if (currentState == TURNNORTHTASK && !(isHeading(Parameters.NORTH))) {
      if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
        stepTurn(Parameters.Direction.LEFT);
      } else {
        stepTurn(Parameters.Direction.RIGHT);
      }
      return;
    }
    if (currentState == TURNNORTHTASK && isHeading(Parameters.NORTH)) {
      currentState = MOVETASK;
      myMove();
      return;
    }
    if (currentState == TURNSOUTHTASK && !(isHeading(Parameters.SOUTH))) {
      if (myGetHeading() < Math.PI / 2 || myGetHeading() > 3 * Math.PI / 2) {
        stepTurn(Parameters.Direction.RIGHT);
      } else {
        stepTurn(Parameters.Direction.LEFT);
      }
      return;
    }
    if (currentState == TURNSOUTHTASK && isHeading(Parameters.SOUTH)) {
      currentState = MOVETASK;
      myMove();
      return;
    }
    if (currentState == TURNEASTTASK && !(isHeading(Parameters.EAST))) {
      if (myGetHeading() < Math.PI && myGetHeading() > 0) {
        stepTurn(Parameters.Direction.LEFT);
      } else {
        stepTurn(Parameters.Direction.RIGHT);
      }
      return;
    }
    if (currentState == TURNEASTTASK && isHeading(Parameters.EAST)) {
      currentState = MOVETASK;
      myMove();
      return;
    }
    if (currentState == TURNWESTTASK && !(isHeading(Parameters.WEST))) {
      if (myGetHeading() < Math.PI && myGetHeading() > 0) {
        stepTurn(Parameters.Direction.RIGHT);
      } else {
        stepTurn(Parameters.Direction.LEFT);
      }
      return;
    }
    if (currentState == TURNWESTTASK && isHeading(Parameters.WEST)) {
      currentState = MOVETASK;
      myMove();
      return;
    }

    if (currentState == SINK) {
      return;
    }
    if (true) {
      return;
    }
  }

  private void myMove() {
    isMovingForward = true;
    move();
  }

  private void myMoveBack() {
    isMovingBackward = true;
    moveBack();
  }

  private double myGetHeading() {
    return normalizeRadian(getHeading());
  }

  private double normalizeRadian(double angle) {
    double result = angle;
    while (result < 0)
      result += 2 * Math.PI;
    while (result >= 2 * Math.PI)
      result -= 2 * Math.PI;
    return result;
  }

  private boolean isSameDirection(double dir1, double dir2) {
    return Math.abs(normalizeRadian(dir1) - normalizeRadian(dir2)) < ANGLEPRECISION;
  }

  private void process(String message) {
    if (Integer.parseInt(message.split(":")[2]) == FIRE) {
      double x = Double.parseDouble(message.split(":")[4]);
      double y = Double.parseDouble(message.split(":")[5]);
      boolean already = false;
      for (ArrayList<Double> list : enemyTargets) {
        if ((Math.abs(x - list.get(1)) <= DISTANCEPRECISION) &&
            (Math.abs(y - list.get(2)) <= DISTANCEPRECISION)) {
          already = true;
        }
      }
      if (!already) {
        ArrayList<Double> target = new ArrayList<Double>(3);
        target.add(Double.parseDouble(message.split(":")[3]));
        target.add(x);
        target.add(y);
        enemyTargets.add(target);
      }
      firingEnabled = true;
    }
    if (Integer.parseInt(message.split(":")[2]) == POSITION) {
      ArrayList<Double> temp = new ArrayList<Double>(2);
      temp.add(Double.parseDouble(message.split(":")[3]));
      temp.add(Double.parseDouble(message.split(":")[4]));
      temp.add(Double.parseDouble(message.split(":")[5]));
      teammates.replace(Integer.parseInt(message.split(":")[0]), temp);
    }
  }

  private void setTarget() {
    ArrayList<ArrayList<Double>> realTargets = new ArrayList<ArrayList<Double>>(5);
    for (ArrayList<Double> target : enemyTargets) {
      if (distance(positionX, positionY, target.get(1), target.get(2)) <= Parameters.bulletRange) {
        realTargets.add(target);
      }
    }

    // Use our new prioritization method
    realTargets = prioritizeTargets(realTargets);

    for (ArrayList<Double> target : realTargets) {
      if (canIShot(target.get(1), target.get(2))) {
        targetX = target.get(1);
        targetY = target.get(2);

        // Adjust firing strategy based on distance and target type
        double targetDist = distance(positionX, positionY, targetX, targetY);

        if (targetDist > Parameters.teamBSecondaryBotFrontalDetectionRange + 100) {
          currentState = STANDINGFIRINGTASK;
        } else if (targetDist < HIGH_THREAT_DISTANCE && target.get(0) == MAIN) {
          // For close main bots, use more aggressive firing
          currentState = BACKWARDFIRINGTASK;
        } else {
          currentState = BACKWARDFIRINGTASK;
        }
        return;
      }
    }
    firingEnabled = false;
  }

  private void firePosition(double x, double y) {
    if (positionX <= x)
      fire(Math.atan((y - positionY) / (double) (x - positionX)));
    else
      fire(Math.PI + Math.atan((y - positionY) / (double) (x - positionX)));
    return;
  }

  private boolean canIShot(double x, double y) {
    // y = a*x+b is the line between the robot and the target
    double a = (y - positionY) / (x - positionX);
    double b = positionY - a * positionX;
    for (ArrayList<Double> ally : teammates.values()) {
      if (distance(positionX, positionY, ally.get(0), ally.get(1)) <= DISTANCEPRECISION) {
        continue; // this ally is this robot
      }
      double angleToAlly = getDirectionToTarget(ally.get(0), ally.get(1));
      double angleToTarget = getDirectionToTarget(x, y);
      if (Math.abs(angleToAlly - angleToTarget) < Math.PI / 12.0
          && distance(positionX, positionY, ally.get(0), ally.get(1)) < distance(positionX, positionY, x, y)) {
        return false;
      }

      if (getHeading() == Parameters.EAST) {
        if (Math.abs(ally.get(1) - positionY) < 15 && ally.get(0) > positionX) {
          return false;
        }
      }
      if (getHeading() == Parameters.WEST) {
        if (Math.abs(ally.get(1) - positionY) < 15 && ally.get(0) < positionX) {
          return false;
        }
      }
      if (getHeading() == Parameters.SOUTH) {
        if (Math.abs(ally.get(0) - positionX) < 15 && ally.get(0) > positionX) {
          return false;
        }
      }
      if (getHeading() == Parameters.NORTH) {
        if (Math.abs(ally.get(0) - positionX) < 15 && ally.get(0) < positionX) {
          return false;
        }
      }

      double allyA = Math.tan(ally.get(2));
      double allyB = ally.get(1) - allyA * ally.get(0);
      double allyX = (b - allyB) / (allyA - a);
      double allyY = a * allyX + b;
      if (distance(ally.get(0), ally.get(1), allyX, allyY) <= SAMELINEPRECISION &&
          (x >= allyX && allyX >= positionX || x <= allyX && allyX <= positionX) &&
          (y >= allyY && allyY >= positionY || y <= allyY && allyY <= positionY)) {
        return false;
      }
    }
    if (detectFront().getObjectType() == IFrontSensorResult.Types.TeamMainBot
        || detectFront().getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot)
      return false;
    return true;
  }

  private double distance(double x1, double y1, double x2, double y2) {
    return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  private void realCoords() {
    positionX = positionX < 0 ? 0 : positionX;
    positionX = positionX > 3000 ? 3000 : positionX;
    positionY = positionY < 0 ? 0 : positionY;
    positionY = positionY > 2000 ? 2000 : positionY;
  }

  private boolean canFireLatency() {
    return cooldownCounter == 0;

  }

  private boolean isHeading(double dir) {
    return Math.abs(Math.sin(getHeading() - dir)) < ANGLEPRECISIONBIS;
  }

  private double getDirectionToTarget(double x, double y) {
    double dir = Math.atan2(y - positionY, x - positionX);
    return normalizeRadian(dir);
  }

  private boolean determineTeam() {
    if (isSameDirection(getHeading(), Parameters.EAST)) {
      return true;
    }
    return false;
  }

  // New method to assess threat level of enemy
  private int assessThreat(IRadarResult enemy) {
    if (enemy.getObjectDistance() < HIGH_THREAT_DISTANCE) {
      return HIGH_PRIORITY;
    } else if (enemy.getObjectDistance() < MEDIUM_THREAT_DISTANCE) {
      return MEDIUM_PRIORITY;
    } else {
      return LOW_PRIORITY;
    }
  }

  // Improved target prioritization
  private ArrayList<ArrayList<Double>> prioritizeTargets(ArrayList<ArrayList<Double>> allTargets) {
    ArrayList<ArrayList<Double>> result = new ArrayList<>(allTargets);

    result.sort(new Comparator<ArrayList<Double>>() {
      @Override
      public int compare(ArrayList<Double> t1, ArrayList<Double> t2) {
        // Priority 1: Target type (Main bots are higher priority)
        if (t1.get(0) != t2.get(0)) {
          return t1.get(0) == MAIN ? -1 : 1;
        }

        // Priority 2: Threat level (if available - index 3)
        if (t1.size() > 3 && t2.size() > 3 && !t1.get(3).equals(t2.get(3))) {
          return t2.get(3).compareTo(t1.get(3)); // Higher threat first
        }

        // Priority 3: Distance (closer targets first)
        double dist1 = distance(positionX, positionY, t1.get(1), t1.get(2));
        double dist2 = distance(positionX, positionY, t2.get(1), t2.get(2));

        // Check if target is the one we've been tracking
        boolean isCurrentTarget1 = (Math.abs(t1.get(1) - lastTargetX) < 50 &&
            Math.abs(t1.get(2) - lastTargetY) < 50);
        boolean isCurrentTarget2 = (Math.abs(t2.get(1) - lastTargetX) < 50 &&
            Math.abs(t2.get(2) - lastTargetY) < 50);

        // Priority 4: Target persistence (prefer continuing to track same target)
        if (isCurrentTarget1 && !isCurrentTarget2) {
          return -1;
        } else if (!isCurrentTarget1 && isCurrentTarget2) {
          return 1;
        }

        // Default to distance comparison
        return Double.compare(dist1, dist2);
      }
    });

    // Update target tracking info if we have targets
    if (!result.isEmpty()) {
      double tx = result.get(0).get(1);
      double ty = result.get(0).get(2);

      // Check if we're still tracking the same target
      if (Math.abs(tx - lastTargetX) < 50 && Math.abs(ty - lastTargetY) < 50) {
        targetLockCount++;
      } else {
        targetLockCount = 0;
      }

      lastTargetX = tx;
      lastTargetY = ty;
    }

    return result;
  }

  // Enhanced method for detecting if path to target is blocked
  private boolean isPathClear(double targetX, double targetY) {
    double angleToTarget = getDirectionToTarget(targetX, targetY);
    double distToTarget = distance(positionX, positionY, targetX, targetY);

    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() != Types.BULLET) {
        double objectAngle = o.getObjectDirection();
        double angleDiff = Math.abs(normalizeRadian(objectAngle - angleToTarget));

        if (angleDiff < 0.2 && o.getObjectDistance() < distToTarget) {
          return false;
        }
      }
    }

    return true;
  }

  // Improved hunting behavior with smarter path selection
  private void huntTarget(double targetX, double targetY) {
    if (isPathClear(targetX, targetY)) {
      // Direct path is clear, move towards target
      double angleToTarget = getDirectionToTarget(targetX, targetY);
      if (Math.abs(myGetHeading() - angleToTarget) < 0.1) {
        myMove();
      } else {
        // Turn towards target
        if (normalizeRadian(angleToTarget - myGetHeading()) < Math.PI) {
          stepTurn(Parameters.Direction.LEFT);
        } else {
          stepTurn(Parameters.Direction.RIGHT);
        }
      }
    } else {
      // Path is blocked, try to find alternate route
      // Simple implementation - turn left and try again next step
      stepTurn(Parameters.Direction.LEFT);
    }
  }

  // --- MÉTHODES DE FORMATION ---
  private double[] calculateFormationPosition(double centerX, double centerY, int robotId) {
    double[] position = new double[2];
    if (myTeam) {
      if (robotId == BETA) {
        position[0] = centerX;
        position[1] = centerY;
      } else if (robotId == GAMMA) {
        position[0] = centerX - TRIANGLE_SIDE;
        position[1] = centerY - (TRIANGLE_SIDE) * Math.sqrt(3) / 2;
      } else if (robotId == ALPHA) {
        position[0] = centerX - TRIANGLE_SIDE;
        position[1] = centerY + (TRIANGLE_SIDE) * Math.sqrt(3) / 2;
      }
    } else { // TEAM_B
      if (robotId == BETA) {
        position[0] = centerX;
        position[1] = centerY;
      } else if (robotId == GAMMA) {
        position[0] = centerX + TRIANGLE_SIDE;
        position[1] = centerY - (TRIANGLE_SIDE) * Math.sqrt(3) / 2;
      } else if (robotId == ALPHA) {
        position[0] = centerX + TRIANGLE_SIDE;
        position[1] = centerY + (TRIANGLE_SIDE) * Math.sqrt(3) / 2;
      }
    }
    return position;
  }

  // --- MÉTHODES DE DÉPLACEMENT ---
  private void moveToCoordinates(double targetX, double targetY) {
    // Pas d'obstacle imminent, continuer vers la cible
    double dx = targetX - positionX;
    double dy = targetY - positionY;
    double distance = Math.hypot(dx, dy);

    if (distance < 30) {
      return; // Destination atteinte
    }

    double angleToTarget = normalizeHeading(Math.atan2(dy, dx));
    turnAndMove(angleToTarget);
  }

  private double normalizeHeading(double angle) {
    while (angle > Math.PI)
      angle -= 2 * Math.PI;
    while (angle < -Math.PI)
      angle += 2 * Math.PI;
    return angle; // Retourne l'angle signé, pas sa valeur absolue
  }

  private double normalizeAngleDifference(double diff) {
    while (diff > Math.PI)
      diff -= 2 * Math.PI;
    while (diff < -Math.PI)
      diff += 2 * Math.PI;
    return diff; // Retourne la différence signée, pas sa valeur absolue
  }

  private void turnAndMove(double desiredAngle) {
    double angleDiff = normalizeAngleDifference(desiredAngle - getHeading());

    if (Math.abs(angleDiff) > Math.PI / 2) {
      double oppositeAngle = normalizeHeading(desiredAngle + Math.PI);
      double newDiff = normalizeAngleDifference(oppositeAngle - getHeading());

      if (Math.abs(newDiff) > ANGLE_PRECISION) {
        stepTurn(newDiff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      } else {
        myMoveBack();
      }
    } else if (Math.abs(angleDiff) > ANGLE_PRECISION) {
      stepTurn(angleDiff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
    } else {
      myMove();
    }
  }
}