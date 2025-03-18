/* ******************************************************
 * Simovies - Eurobot 5 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/Stage1.java 2014-10-18 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

import java.util.ArrayList;

public class teamMain extends Brain {

  // --- CONSTANTES ---
  // Précision
  private static final double ANGLE_PRECISION = 0.05;
  private static final double FIRE_ANGLE_PRECISION = Math.PI / 6.0;

  // Identifiants de robot
  private static final int ALPHA = 0x1EADDA;
  private static final int BETA = 0x5EC0;
  private static final int GAMMA = 0x333;
  private static final int TEAM = 0xBADDAD;
  private static final int UNDEFINED = 0xBADC0DE0;

  // Types de messages
  private static final int FIRE = 0xB52;
  private static final int FALLBACK = 0xFA11BAC;
  private static final int ROGER = 0x0C0C0C0C;
  private static final int COMBAT = 0xB52B52;
  private static final int OVER = 0xC00010FF;
  private static final int DODGE = 0xD0D6E;

  // États de tâches
  private static final int TURNSOUTHTASK = 1;
  private static final int MOVETASK = 2;
  private static final int TURNLEFTTASK = 3;
  private static final int SINK = 0xBADC0DE1;
  
  // États de formation
  private static final int TRIANGLE_FORMATION = 4;
  private static final int MOVE_TO_POSITION = 5;
  private static final int TURN_TO_POSITION = 6;
  private static final int FINAL_ORIENTATION = 7;
  private static final int ENEMY_ENGAGEMENT = 8;

  // Équipes
  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  // Paramètres de formation
  private static final double TRIANGLE_SIDE = 250;  // Distance entre robots

  // Paramètres d'odométrie
  private static final double COLLISION_THRESHOLD = 50;
  private static final double POSITION_CONFIDENCE = 0.9;
  private static final int RECALIBRATION_INTERVAL = 100;
  private static final double ARENA_WIDTH = 3000;
  private static final double ARENA_HEIGHT = 2000;

  // Paramètres de formation de combat
  private static final int MAX_ENGAGEMENT_TIME = 100;
  private static final double COMBAT_TRIANGLE_DISTANCE = 200;

  // --- VARIABLES ---
  private int state;
  private double oldAngle;
  private double myX, myY;
  private boolean isMoving;
  private boolean isMovingBack;
  private int whoAmI;
  private int fireRythm, rythm, counter;
  private int countDown;
  private double targetX, targetY;
  private boolean fireOrder;
  private boolean freeze;
  private boolean friendlyFire;
  private ArrayList<String> receivedMessages;
  private int myTeam;
  
  // Variables pour la formation
  private double formationX, formationY;
  private boolean positionReached = false;
  private boolean readyAlpha = false;
  private boolean readyBeta = false;
  private boolean readyGamma = false;
  
  // Variables d'odométrie
  private double estimatedErrorX = 0;
  private double estimatedErrorY = 0;
  private double lastKnownX, lastKnownY;
  private int stepsSinceRecalibration = 0;
  private double[] lastMovement = {0, 0};
  private boolean collisionDetected = false;
  
  // Variables d'engagement ennemi
  private double lastEnemyX = 0;
  private double lastEnemyY = 0;
  private int enemyEngagementCounter = 0;
  private boolean combatFormationReady = false;
  
  // --- CONSTRUCTEUR ---
  public teamMain() {
    super();
  }

  // --- INITIALISATION ---
  public void activate() {
    determineInitialPosition();
    myTeam = determineTeam();
    
    // Initialisation des variables
    state = TRIANGLE_FORMATION;
    isMoving = false;
    fireOrder = false;
    fireRythm = 0;
    oldAngle = getHeading();
    receivedMessages = new ArrayList<>();
    
    // Définir la position cible de la formation triangulaire
    double centerX = myX + (myTeam == TEAM_A ? 300 : -300);
    double centerY = myY;
    double[] formationPosition = calculateFormationPosition(centerX, centerY, whoAmI);
    formationX = formationPosition[0];
    formationY = formationPosition[1];
    
    // Initialisation pour l'odométrie
    lastKnownX = myX;
    lastKnownY = myY;
  }
  
  /**
   * Détermine la position initiale et l'identité du robot à partir des détections radar.
   */
  private void determineInitialPosition() {
    whoAmI = GAMMA;
    for (IRadarResult o : detectRadar()) {
      if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
        whoAmI = ALPHA;
    }
    for (IRadarResult o : detectRadar()) {
      if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH) && whoAmI != GAMMA)
        whoAmI = BETA;
    }
    if (whoAmI == GAMMA) {
      myX = Parameters.teamAMainBot1InitX;
      myY = Parameters.teamAMainBot1InitY;
    } else if (whoAmI == BETA) {
      myX = Parameters.teamAMainBot2InitX;
      myY = Parameters.teamAMainBot2InitY;
    } else if (whoAmI == ALPHA) {
      myX = Parameters.teamAMainBot3InitX;
      myY = Parameters.teamAMainBot3InitY;
    }
  }
  
  // --- BOUCLE PRINCIPALE ---
  public void step() {
    updateOdometry();
    if (getHealth() < 1) {
      state = SINK;
      return;
    }
    
    // Mise à jour de la position selon le déplacement effectué
    if (isMoving) {
      myX += Parameters.teamAMainBotSpeed * Math.cos(getHeading());
      myY += Parameters.teamAMainBotSpeed * Math.sin(getHeading());
      isMoving = false;
    } else if (isMovingBack) {
      myX -= Parameters.teamAMainBotSpeed * Math.cos(getHeading());
      myY -= Parameters.teamAMainBotSpeed * Math.sin(getHeading());
      isMovingBack = false;
    }
    
    logDebugMessages();
    
    // Communication : traiter les messages entrants
    processIncomingMessages();
    
    // Détection radar et engagement ennemi
    checkTeamCollision();
    checkEnnemie();
    freeze = false;
    friendlyFire = true;
    for (IRadarResult o : detectRadar()) {
      // Détection d'ennemis pour le tir
      if (o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
          o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
        double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        sendMessage(FIRE, enemyX, enemyY);
      }
      // Arrêt en cas d'obstacle latéral
      if (o.getObjectDistance() <= 100 && 
          !isRoughlySameDirection(o.getObjectDirection(), getHeading()) && 
          o.getObjectType() != IRadarResult.Types.BULLET) {
        freeze = true;
      }
      // Gestion du risque de tir ami
      if (o.getObjectType() == IRadarResult.Types.TeamMainBot ||
          o.getObjectType() == IRadarResult.Types.TeamSecondaryBot ||
          o.getObjectType() == IRadarResult.Types.Wreck) {
        if (fireOrder && onTheWay(o.getObjectDirection())) {
          friendlyFire = false;
        }
      }
    }
    if (freeze) return;
    
    // Gestion du tir
    if (fireOrder) countDown++;
    if (countDown >= 100) fireOrder = false;
    if (fireOrder && fireRythm == 0 && friendlyFire) {
      firePosition(targetX, targetY);
      fireRythm++;
      return;
    }
    fireRythm = (fireRythm + 1) % Parameters.bulletFiringLatency;
    
    // Gestion de l'état d'engagement ennemi
    if (state == ENEMY_ENGAGEMENT) {
      handleEnemyEngagement();
      return;
    }
    
    // Formation triangulaire
    if (state == TRIANGLE_FORMATION) {
      double distToTarget = Math.hypot(myX - formationX, myY - formationY);
      if (distToTarget < 30) {
        positionReached = true;
        state = FINAL_ORIENTATION;
        return;
      } else {
        moveToCoordinates(formationX, formationY);
        return;
      }
    }
    
    // Orientation finale selon l'équipe
    if (state == FINAL_ORIENTATION) {
      double targetDirection = (myTeam == TEAM_A) ? Parameters.EAST : Parameters.WEST;
      if (isSameDirection(getHeading(), targetDirection)) {
        sendMessage(ROGER, true);
        if (readyAlpha && readyBeta && readyGamma) {
          readyAlpha = readyBeta = readyGamma = false;
          state = MOVETASK;
        }
        return;
      } else {
        double angleDiff = normalizeAngle(targetDirection - getHeading());
        if (angleDiff < Math.PI)
          stepTurn(Parameters.Direction.RIGHT);
        else
          stepTurn(Parameters.Direction.LEFT);
        return;
      }
    }
    
    // Détection d'obstacles (épaves) avec le radar
    for (IRadarResult o : detectRadar()) {
      if (state != DODGE && o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 150) {
        state = DODGE;
        double obstacleX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double obstacleY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        sendMessage(DODGE, obstacleX, obstacleY);
        sendLogMessage("DODGE state activated. Wreck detected at (" + obstacleX + ", " + obstacleY + ")");
        return;
      }
    }
    
    // Gestion de l'état DODGE
    if (state == DODGE) {
      myMoveBack();
      boolean tooClose = false;
      for (IRadarResult o : detectRadar()) {
        if (o.getObjectDistance() < 500 && o.getObjectType() == IRadarResult.Types.Wreck) {
          double obstacleX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
          double obstacleY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
          sendMessage(DODGE, obstacleX, obstacleY);
          tooClose = true;
          break;
        }
      }
      if (!tooClose) {
        state = TRIANGLE_FORMATION;
      }
      return;
    }
    
    // États de déplacement simples
    if (state == TURNSOUTHTASK) {
      if (!isSameDirection(getHeading(), Parameters.SOUTH)) {
        stepTurn(Parameters.Direction.RIGHT);
        return;
      } else {
        state = MOVETASK;
        myMove();
        return;
      }
    }
    
    if (state == MOVETASK) {
      boolean obstacleAhead = detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
      for (IRadarResult o : detectRadar()) {
        if (isRoughlySameDirection(o.getObjectDirection(), getHeading()) &&
            o.getObjectDistance() <= 50) {
          obstacleAhead = true;
          break;
        }
      }
      if (obstacleAhead) {
        state = TURNLEFTTASK;
        oldAngle = getHeading();
        stepTurn(Parameters.Direction.LEFT);
        return;
      } else {
        myMove();
        return;
      }
    }
    
    if (state == TURNLEFTTASK) {
      if (!isSameDirection(getHeading(), oldAngle + Parameters.LEFTTURNFULLANGLE)) {
        stepTurn(Parameters.Direction.LEFT);
        return;
      } else {
        state = MOVETASK;
        myMove();
        return;
      }
    }
    
    if (state == FIRE) {
      if (fireRythm == 0) {
        firePosition(700, 1500);
        fireRythm++;
        return;
      }
      fireRythm++;
      if (fireRythm == Parameters.bulletFiringLatency) fireRythm = 0;
      if (rythm == 0) stepTurn(Parameters.Direction.LEFT);
      else myMove();
      rythm = (rythm + 1) % 14;
      return;
    }
    
    if (state == SINK) {
      sendLogMessage("I'm dead");
      return;
    }
  }
  
  // --- GESTION DE L'ENGAGEMENT ENNEMI ---
  private void handleEnemyEngagement() {
    boolean enemyStillPresent = false;
    double closestEnemyDistance = Double.MAX_VALUE;
    double enemyDirection = 0;
    
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == IRadarResult.Types.OpponentMainBot || 
          o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
        enemyStillPresent = true;
        if (o.getObjectDistance() < closestEnemyDistance) {
          closestEnemyDistance = o.getObjectDistance();
          enemyDirection = o.getObjectDirection();
          lastEnemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
          lastEnemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
          sendMessage(FIRE, lastEnemyX, lastEnemyY, myX, myY);
          double[] combatPosition = calculateCombatFormation(lastEnemyX, lastEnemyY, whoAmI);
          formationX = combatPosition[0];
          formationY = combatPosition[1];
        }
      }
    }
    
    if (!enemyStillPresent) {
      enemyEngagementCounter++;
      if (enemyEngagementCounter > MAX_ENGAGEMENT_TIME) {
        sendLogMessage("Enemy lost or eliminated. Returning to normal operations.");
        state = MOVETASK;
        return;
      }
    } else {
      enemyEngagementCounter = 0;
    }
    
    double distToPosition = Math.hypot(myX - formationX, myY - formationY);
    if (distToPosition < 30) {
      combatFormationReady = true;
      double angleToEnemy = Math.atan2(lastEnemyY - myY, lastEnemyX - myX);
      if (!isRoughlySameDirection(getHeading(), angleToEnemy)) {
        double angleDiff = normalizeAngle(angleToEnemy - getHeading());
        if (angleDiff < Math.PI)
          stepTurn(Parameters.Direction.RIGHT);
        else
          stepTurn(Parameters.Direction.LEFT);
      } else if (whoAmI == BETA || closestEnemyDistance < 300) {
        if (fireRythm == 0 && friendlyFire) {
          firePosition(lastEnemyX, lastEnemyY);
          fireRythm++;
          sendLogMessage("Firing at enemy from formation position");
        }
      }
    } else {
      moveToCoordinates(formationX, formationY);
      sendLogMessage("Moving to combat position: " + (int) distToPosition + " units away");
    }
  }
  
  // --- MÉTHODES DE DÉPLACEMENT ---
  private boolean moveToCoordinates(double targetX, double targetY) {
    double xDiff = targetX - myX;
    double yDiff = targetY - myY;
    
    if (checkTeamCollision()) return false;
    
    // Aligner d'abord sur l'axe Y (Nord-Sud)
    if (Math.abs(yDiff) > 20) {
      double targetDirection = (yDiff > 0) ? Parameters.SOUTH : Parameters.NORTH;
      if (!isSameDirection(getHeading(), targetDirection)) {
        if (normalizeAngle(targetDirection - getHeading()) < Math.PI)
          stepTurn(Parameters.Direction.RIGHT);
        else
          stepTurn(Parameters.Direction.LEFT);
        return false;
      } else {
        myMove();
        return true;
      }
    }
    // Puis sur l'axe X (Est-Ouest)
    else if (Math.abs(xDiff) > 20) {
      double targetDirection = (xDiff > 0) ? Parameters.EAST : Parameters.WEST;
      if (!isSameDirection(getHeading(), targetDirection)) {
        if (normalizeAngle(targetDirection - getHeading()) < Math.PI)
          stepTurn(Parameters.Direction.RIGHT);
        else
          stepTurn(Parameters.Direction.LEFT);
        return false;
      } else {
        myMove();
        return true;
      }
    }
    return false;
  }
  
  private void myMove() {
    isMoving = true;
    move();
  }
  
  private void myMoveBack() {
    isMovingBack = true;
    moveBack();
  }
  
  // --- MÉTHODES UTILITAIRES ---
  private double normalizeAngle(double angle) {
    while (angle > Math.PI)
      angle -= 2 * Math.PI;
    while (angle < -Math.PI)
      angle += 2 * Math.PI;
    return Math.abs(angle);
  }
  
  private boolean isSameDirection(double dir1, double dir2) {
    return normalizeAngle(dir1 - dir2) < ANGLE_PRECISION;
  }
  
  private boolean isRoughlySameDirection(double dir1, double dir2) {
    return Math.abs(normalizeAngle(dir1) - normalizeAngle(dir2)) < FIRE_ANGLE_PRECISION;
  }
  
  private void firePosition(double x, double y) {
    if (myX <= x)
      fire(Math.atan((y - myY) / (x - myX)));
    else
      fire(Math.PI + Math.atan((y - myY) / (x - myX)));
  }
  
  private boolean onTheWay(double angle) {
    if (myX <= targetX)
      return isRoughlySameDirection(angle, Math.atan((targetY - myY) / (targetX - myX)));
    else
      return isRoughlySameDirection(angle, Math.PI + Math.atan((targetY - myY) / (targetX - myX)));
  }
  
  private int determineTeam() {
    return isSameDirection(getHeading(), Parameters.EAST) ? TEAM_A : TEAM_B;
  }
  
  // --- MÉTHODES DE COMMUNICATION ---
  private void sendMessage(int messageType, Object... data) {
    StringBuilder message = new StringBuilder();
    message.append(whoAmI).append(":").append(TEAM).append(":").append(messageType);
    if (data != null) {
      for (Object datum : data) {
        message.append(":").append(datum);
      }
    }
    message.append(":").append(OVER);
    broadcast(message.toString());
  }
  
  private void process(String message) {
    String[] parts = message.split(":");
    int messageType = Integer.parseInt(parts[2]);
    
    switch (messageType) {
      case FIRE:
        fireOrder = true;
        countDown = 0;
        targetX = Double.parseDouble(parts[3]);
        targetY = Double.parseDouble(parts[4]);
        if (parts.length >= 7) {
          double senderX = Double.parseDouble(parts[5]);
          double senderY = Double.parseDouble(parts[6]);
          if (state != TRIANGLE_FORMATION && state != FINAL_ORIENTATION) {
            state = ENEMY_ENGAGEMENT;
            lastEnemyX = targetX;
            lastEnemyY = targetY;
            enemyEngagementCounter = 0;
            combatFormationReady = false;
            double[] combatPosition = calculateCombatFormation(targetX, targetY, whoAmI);
            formationX = combatPosition[0];
            formationY = combatPosition[1];
            sendLogMessage("Combat formation: moving to (" + (int) formationX + "," + (int) formationY + ")");
          }
        }
        break;
        
      case FALLBACK:
        // Gestion du message de repli (à implémenter si nécessaire)
        break;
        
      case ROGER:
        int whoAreYou = Integer.parseInt(parts[0]);
        boolean ready = Boolean.parseBoolean(parts[3]);
        if (whoAreYou == ALPHA) readyAlpha = ready;
        if (whoAreYou == BETA) readyBeta = ready;
        if (whoAreYou == GAMMA) readyGamma = ready;
        break;
        
      case DODGE:
        double obstacleX = Double.parseDouble(parts[3]);
        double obstacleYtmp = Double.parseDouble(parts[4]);
        double betaX = myX;
        double betaY = myY;
        if (parts.length >= 7) {
          betaX = Double.parseDouble(parts[5]);
          betaY = Double.parseDouble(parts[6]);
        } else if (whoAmI == BETA) {
          sendMessage(DODGE, obstacleX, obstacleYtmp, betaX, betaY);
        } else {
          return;
        }
        double obstacleY = obstacleYtmp < betaY ? betaY + 500 : betaY - 500;
        double[] pos = calculateFormationPosition(obstacleX, obstacleY, whoAmI);
        formationX = pos[0];
        formationY = pos[1];
        state = DODGE;
        break;
        
      default:
        sendLogMessage("Unknown message type: " + messageType);
        break;
    }
  }
  
  private void processIncomingMessages() {
    ArrayList<String> messages = fetchAllMessages();
    receivedMessages.clear();
    for (String m : messages) {
      String[] parts = m.split(":");
      if (Integer.parseInt(parts[1]) == whoAmI || Integer.parseInt(parts[1]) == TEAM) {
        receivedMessages.add(m);
        process(m);
      }
    }
  }
  
  private boolean checkTeamCollision() {
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == IRadarResult.Types.TeamMainBot ||
          o.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
        if (o.getObjectDistance() <= 60 && onTheWay(o.getObjectDirection())) {
          return true;
        }
      }
    }
    return false;
  }
  
  // --- MÉTHODES DE FORMATION ---
  private double[] calculateFormationPosition(double centerX, double centerY, int robotId) {
    double[] position = new double[2];
    if (robotId == BETA) {
      position[0] = centerX;
      position[1] = centerY;
    } else if (robotId == GAMMA) {
      position[0] = centerX - TRIANGLE_SIDE;
      position[1] = centerY - (TRIANGLE_SIDE * 0.75) * Math.sqrt(3) / 2;
    } else if (robotId == ALPHA) {
      position[0] = centerX - TRIANGLE_SIDE;
      position[1] = centerY + (TRIANGLE_SIDE * 0.75) * Math.sqrt(3) / 2;
    }
    return position;
  }
  
  private double[] calculateCombatFormation(double enemyX, double enemyY, int robotId) {
    double[] position = new double[2];
    double directionX = enemyX - myX;
    double directionY = enemyY - myY;
    double distance = Math.hypot(directionX, directionY);
    if (distance > 0) {
      directionX /= distance;
      directionY /= distance;
    } else {
      directionX = 1;
      directionY = 0;
    }
    double safeDistance = (getHealth() < 0.5) ? 300 : 200;
    
    if (robotId == BETA) {
      position[0] = enemyX - directionX * safeDistance;
      position[1] = enemyY - directionY * safeDistance;
    } else {
      double perpX = -directionY;
      double perpY = directionX;
      double betaX = enemyX - directionX * safeDistance;
      double betaY = enemyY - directionY * safeDistance;
      double flankDistance = (getHealth() > 0.7) ? COMBAT_TRIANGLE_DISTANCE * 0.8 : COMBAT_TRIANGLE_DISTANCE;
      if (robotId == ALPHA) {
        position[0] = betaX - directionX * 100 + perpX * flankDistance;
        position[1] = betaY - directionY * 100 + perpY * flankDistance;
      } else { // GAMMA
        position[0] = betaX - directionX * 100 - perpX * flankDistance;
        position[1] = betaY - directionY * 100 - perpY * flankDistance;
      }
    }
    position[0] = Math.min(Math.max(position[0], 100), ARENA_WIDTH - 100);
    position[1] = Math.min(Math.max(position[1], 100), ARENA_HEIGHT - 100);
    return position;
  }
  
  // --- MÉTHODES D'ODOMÉTRIE ET DE DÉTECTION DE COLLISION ---
  private void updateOdometry() {
    boolean collision = detectCollision();
    double plannedDX = 0, plannedDY = 0;
    
    if (isMoving) {
      plannedDX = Parameters.teamAMainBotSpeed * Math.cos(getHeading());
      plannedDY = Parameters.teamAMainBotSpeed * Math.sin(getHeading());
      if (!collision) {
        myX += plannedDX;
        myY += plannedDY;
        lastMovement[0] = plannedDX;
        lastMovement[1] = plannedDY;
      } else {
        myX += plannedDX * 0.3;
        myY += plannedDY * 0.3;
        estimatedErrorX += plannedDX * 0.7;
        estimatedErrorY += plannedDY * 0.7;
        sendLogMessage("Collision detected! Position may be inaccurate.");
      }
      isMoving = false;
    } else if (isMovingBack) {
      plannedDX = -Parameters.teamAMainBotSpeed * Math.cos(getHeading());
      plannedDY = -Parameters.teamAMainBotSpeed * Math.sin(getHeading());
      if (!collision) {
        myX += plannedDX;
        myY += plannedDY;
        lastMovement[0] = plannedDX;
        lastMovement[1] = plannedDY;
      } else {
        myX += plannedDX * 0.3;
        myY += plannedDY * 0.3;
        estimatedErrorX += plannedDX * 0.7;
        estimatedErrorY += plannedDY * 0.7;
        sendLogMessage("Collision detected during backward movement!");
      }
      isMovingBack = false;
    }
    
    estimatedErrorX *= 1.01;
    estimatedErrorY *= 1.01;
    
    stepsSinceRecalibration++;
    if (stepsSinceRecalibration >= RECALIBRATION_INTERVAL) {
      recalibratePosition();
      stepsSinceRecalibration = 0;
    }
    refinePositionWithRadar();
  }
  
  private boolean detectCollision() {
    if (detectFront().getObjectType() != IFrontSensorResult.Types.NOTHING && isMoving) {
      collisionDetected = true;
      return true;
    }
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectDistance() < COLLISION_THRESHOLD &&
          isRoughlySameDirection(o.getObjectDirection(), getHeading())) {
        collisionDetected = true;
        return true;
      }
    }
    collisionDetected = false;
    return false;
  }
  
  private void recalibratePosition() {
    if (Math.hypot(estimatedErrorX, estimatedErrorY) > 100) {
      for (IRadarResult o : detectRadar()) {
        if (o.getObjectType() == IRadarResult.Types.Wreck) {
          double objectDistance = o.getObjectDistance();
          if (objectDistance < 300) {
            double wreckX = myX + objectDistance * Math.cos(o.getObjectDirection());
            double wreckY = myY + objectDistance * Math.sin(o.getObjectDirection());
            if (isNearMapBoundary(wreckX, wreckY)) {
              double correctionX = (wreckX < ARENA_WIDTH / 4) ? wreckX - 100 : 
                                     (wreckX > 3 * ARENA_WIDTH / 4) ? wreckX + 100 : wreckX;
              double correctionY = (wreckY < ARENA_HEIGHT / 4) ? wreckY - 100 : 
                                     (wreckY > 3 * ARENA_HEIGHT / 4) ? wreckY + 100 : wreckY;
              myX = myX * 0.3 + correctionX * 0.7;
              myY = myY * 0.3 + correctionY * 0.7;
              estimatedErrorX = 0;
              estimatedErrorY = 0;
              sendLogMessage("Position recalibrated using wreck reference at (" + (int) wreckX + "," + (int) wreckY + ")");
            }
          }
        }
        double objectDistance = o.getObjectDistance();
        if (objectDistance < 200) {
          double objectX = myX + objectDistance * Math.cos(o.getObjectDirection());
          double objectY = myY + objectDistance * Math.sin(o.getObjectDirection());
          if (isVeryCloseToMapBoundary(objectX, objectY)) {
            snapToMapBoundary(objectDistance, o.getObjectDirection());
            estimatedErrorX = 0;
            estimatedErrorY = 0;
            sendLogMessage("Position recalibrated using boundary reference.");
          }
        }
      }
      if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
        double wallDistance = Parameters.teamAMainBotFrontalDetectionRange;
        double wallX = myX + wallDistance * Math.cos(getHeading());
        double wallY = myY + wallDistance * Math.sin(getHeading());
        if (isVeryCloseToMapBoundary(wallX, wallY)) {
          if (isRoughlySameDirection(getHeading(), 0))
            myX = ARENA_WIDTH - wallDistance - Parameters.teamAMainBotRadius;
          else if (isRoughlySameDirection(getHeading(), Math.PI))
            myX = wallDistance + Parameters.teamAMainBotRadius;
          else if (isRoughlySameDirection(getHeading(), Math.PI / 2))
            myY = ARENA_HEIGHT - wallDistance - Parameters.teamAMainBotRadius;
          else if (isRoughlySameDirection(getHeading(), -Math.PI / 2))
            myY = wallDistance + Parameters.teamAMainBotRadius;
          estimatedErrorX = 0;
          estimatedErrorY = 0;
          sendLogMessage("Position recalibrated using front wall detection.");
        }
      }
    }
  }
  
  private boolean isVeryCloseToMapBoundary(double x, double y) {
    return (x < 20 || x > ARENA_WIDTH - 20 || y < 20 || y > ARENA_HEIGHT - 20);
  }
  
  private boolean isNearMapBoundary(double x, double y) {
    return (x < 50 || x > ARENA_WIDTH - 50 || y < 50 || y > ARENA_HEIGHT - 50);
  }
  
  private void snapToMapBoundary(double wallDistance, double wallDirection) {
    double normalizedDirection = normalizeAngle(wallDirection);
    if (isRoughlySameDirection(normalizedDirection, 0))
      myX = ARENA_WIDTH - wallDistance;
    else if (isRoughlySameDirection(normalizedDirection, Math.PI))
      myX = wallDistance;
    else if (isRoughlySameDirection(normalizedDirection, Math.PI / 2))
      myY = ARENA_HEIGHT - wallDistance;
    else if (isRoughlySameDirection(normalizedDirection, -Math.PI / 2))
      myY = wallDistance;
  }
  
  private void refinePositionWithRadar() {
    // Utiliser les objets fixes détectés pour affiner notre position
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == IRadarResult.Types.Wreck) {
        double wreckX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double wreckY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        // Une logique de raffinement pourrait être ajoutée ici
      }
    }
  }
  
  // --- MÉTHODES DE DÉBOGAGE ---
  private void logDebugMessages() {
    boolean debug = true;
    if (debug && whoAmI == ALPHA && state != SINK) {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("ALPHAState=" + state + " current=(" + (int) myX + ", " + (int) myY + ") target=(" + (int) formationX + ", " + (int) formationY + ")");
    }
    if (debug && whoAmI == BETA && state != SINK) {
      sendLogMessage("#BETAState=" + state + " current=(" + (int) myX + ", " + (int) myY + ") target=(" + (int) formationX + ", " + (int) formationY + ")");
    }
    if (debug && whoAmI == GAMMA && state != SINK) {
      sendLogMessage("#GAMMAState=" + state + " current=(" + (int) myX + ", " + (int) myY + ") target=(" + (int) formationX + ", " + (int) formationY + ")");
    }
  }

  /**
 * Vérifie si un ennemi est détecté par le radar.
 * @return true si un ennemi est détecté, false sinon.
 */
private boolean checkEnnemie() {
  for (IRadarResult o : detectRadar()) {
    if (o.getObjectType() == IRadarResult.Types.OpponentMainBot ||
        o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
      double distance = o.getObjectDistance();
      if (distance < 500) {  // Seuil de détection
        lastEnemyX = myX + distance * Math.cos(o.getObjectDirection());
        lastEnemyY = myY + distance * Math.sin(o.getObjectDirection());
        sendMessage(FIRE, lastEnemyX, lastEnemyY, myX, myY);
        if (state != TRIANGLE_FORMATION && state != FINAL_ORIENTATION) {
          sendLogMessage("Enemy detected! Switching to combat formation.");
          state = ENEMY_ENGAGEMENT;
          enemyEngagementCounter = 0;
          combatFormationReady = false;
        }
        return true;
      }
    }
  }
  return false;
}

}
