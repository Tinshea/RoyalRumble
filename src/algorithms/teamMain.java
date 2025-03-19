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
import java.util.HashMap;
import java.util.Map;

public class teamMain extends Brain {

  // --- CONSTANTES ---
  // Précision
  private static final double ANGLE_PRECISION = 0.01; 
  private static final double FIRE_ANGLE_PRECISION = Math.PI / 6.0;

  // Identifiants de robot
  private static final int ALPHA = 0x1EADDA;
  private static final int BETA = 0x5EC0;
  private static final int GAMMA = 0x333;
  private static final int TEAM = 0xBADDAD;

  // Types de messages
  private static final int FIRE = 0xB52;
  private static final int ROGER = 0x0C0C0C0C;
  private static final int OVER = 0xC00010FF;
  private static final int DODGE = 0xD0D6E;

  // États de tâches
  private static final int SINK = 0xBADC0DE1;
  private static final int MOVETASK = 2;
  private static final int TURNLEFTTASK = 3;
  private static final int TRIANGLE_FORMATION = 4;
  private static final int FINAL_ORIENTATION = 7;
  
  // Sous-états pour TURNLEFTTASK
  private static final int TURNLEFT_BACKWARD = 0;
  private static final int TURNLEFT_ROTATE = 1;
  private static final int TURNLEFT_COMPLETE = 2;

  // Équipes
  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  // Paramètres de formation et carte
  private static final double TRIANGLE_SIDE = 200;
  private static final double COLLISION_THRESHOLD = 150;
  private static final double ARENA_WIDTH = 3000;
  private static final double ARENA_HEIGHT = 2000;
  private static final double SAFETY_MARGIN = 100.0;
  private static final double ENEMY_DETECTION_THRESHOLD = 200;
  
  // Paramètres de tir améliorés
  private static final int FIRE_COOLDOWN = Parameters.bulletFiringLatency - 2; // Tire plus fréquemment
  private static final double MAX_FIRING_DISTANCE = 1000.0; // Distance max de tir efficace
  private static final long ENEMY_TRACK_TIMEOUT = 3000; // Temps en ms pour oublier un ennemi non détecté

  // --- VARIABLES ---
  // État et position
  private int state;
  private double oldAngle;
  private double myX, myY;
  private int whoAmI;
  private int myTeam;
  
  // Contrôle de mouvement
  private boolean isMoving;
  private boolean isMovingBack;
  private int turnLeftSubState = TURNLEFT_BACKWARD; // Sous-état pour la tâche TURNLEFTTASK
  private int backwardStepCount = 0; // Compteur pour les étapes de recul
  private double targetRotation; // Angle cible pour la rotation
  
  // Gestion du tir
  private int fireRythm;
  private int countDown;
  private double targetX, targetY;
  private boolean fireOrder;
  private boolean freeze;
  private boolean friendlyFire;
  
  // Formation
  private double formationX, formationY;
  private boolean readyAlpha = false;
  private boolean readyBeta = false;
  private boolean readyGamma = false;
  
  // Détection d'ennemis
  private double lastEnemyX = 0;
  private double lastEnemyY = 0;
  
  // Communication
  private ArrayList<String> receivedMessages;
  
  // Suivi des ennemis - Nouvelle structure pour le tracking amélioré
  private Map<Integer, EnemyInfo> enemyTracker = new HashMap<>();
  private int targetEnemyId = -1;
  private long lastScanTime = 0;
  
  // --- CONSTRUCTEUR ---
  public teamMain() {
    super();
  }
  
  // Nouvelle classe pour stocker les informations de suivi des ennemis
  private class EnemyInfo {
    public double x;
    public double y;
    public double lastX;
    public double lastY;
    public long lastSeen;
    public double distance;
    public IRadarResult.Types type;
    
    public EnemyInfo(double x, double y, double distance, IRadarResult.Types type) {
      this.x = x;
      this.y = y;
      this.lastX = x;
      this.lastY = y;
      this.lastSeen = System.currentTimeMillis();
      this.distance = distance;
      this.type = type;
    }
    
    // Mettre à jour la position et calculer la direction et vitesse estimées
    public void update(double newX, double newY, double distance) {
      this.lastX = this.x;
      this.lastY = this.y;
      this.x = newX;
      this.y = newY;
      this.distance = distance;
      this.lastSeen = System.currentTimeMillis();
    }
    
    // Estimer la position future pour améliorer la précision du tir
    public double[] predictPosition(int timeSteps) {
      // Vecteur de déplacement
      double dx = x - lastX;
      double dy = y - lastY;
      
      // Position prédite
      return new double[] {
        x + dx * timeSteps,
        y + dy * timeSteps
      };
    }
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
    
    // Définition de la position cible
    initializeFormationPosition();
  }
  
  private void initializeFormationPosition() {
    double centerX = Parameters.teamAMainBot2InitX + (myTeam == TEAM_A ? 300 : -300);
    double centerY = Parameters.teamAMainBot2InitY;
    double[] formationPosition = calculateFormationPosition(centerX, centerY, whoAmI);
    formationX = formationPosition[0]; 
    formationY = formationPosition[1];
  }
  
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
    
    // Définir la position initiale selon l'identité du robot
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
    // Vérification de la santé
    if (getHealth() < 1) {
      state = SINK;
    }
    
    logDebugMessages();
    processIncomingMessages();
    
    // Gestion de la détection et du tir
    handleRadarDetection();
    if (freeze) return;
    
    // Logique de tir améliorée
    if (fireRythm == 0 && targetEnemyId != -1 && friendlyFire) {
      EnemyInfo target = enemyTracker.get(targetEnemyId);
      if (target != null && target.distance < MAX_FIRING_DISTANCE) {
        // Utiliser la prédiction de position pour augmenter la précision
        double[] predictedPos = target.predictPosition(2);
        firePosition(predictedPos[0], predictedPos[1]);
        fireRythm++;
        return;
      }
    }
    
    // Gérer le cooldown du tir
    if (fireRythm > 0) {
      fireRythm++;
      if (fireRythm >= FIRE_COOLDOWN) fireRythm = 0;
    }
    
    // Gestion des états
    switch (state) {
      case TRIANGLE_FORMATION:
        handleTriangleFormation();
        break;
      case FINAL_ORIENTATION:
        handleFinalOrientation();
        break;
      case DODGE:
        handleDodgeState();
        break;
      case MOVETASK:
        handleMoveTask();
        break;
      case TURNLEFTTASK:
        handleTurnLeftTask();
        break;
    
      case SINK:
        sendMessage(ROGER, true);
        sendLogMessage("I'm dead");
        return;
        
    }
    
    // Nettoyer les ennemis qui n'ont pas été détectés pendant un certain temps
    cleanupStaleEnemies();
  }
  
  // --- HANDLERS POUR LES DIFFÉRENTS ÉTATS ---
  private void handleTriangleFormation() {
    double distToTarget = Math.hypot(myX - formationX, myY - formationY);
    if (distToTarget < 30) {
      state = FINAL_ORIENTATION;
    } else {
      moveToCoordinates(formationX, formationY);
    }
  }
  
  private void handleFinalOrientation() {
    double targetDirection = (myTeam == TEAM_A) ? Parameters.EAST : Parameters.WEST;
    if (isSameDirection(getHeading(), targetDirection)) {
      if ((whoAmI == ALPHA && !readyAlpha) || (whoAmI == BETA && !readyBeta) || (whoAmI == GAMMA && !readyGamma)) {
        sendMessage(ROGER, true);
      }

      if (readyAlpha && readyBeta && readyGamma) {
        readyAlpha = readyBeta = readyGamma = false; 
        state = MOVETASK;
      }
    } else {
      turnTowards(targetDirection);
    }
  }
  
  private void handleDodgeState() {
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
  }
  
  private void handleMoveTask() {
    if (isObstacleAhead()) {
      state = TURNLEFTTASK;
      oldAngle = getHeading();
      // Initialisation du sous-état TURNLEFTTASK
      turnLeftSubState = TURNLEFT_BACKWARD;
      backwardStepCount = 0;
      // Définir l'angle cible (opposé à la direction actuelle)
      targetRotation = normalizeHeading(getHeading() + Math.PI);
      sendLogMessage("Obstacle ahead, initiating turn maneuver. Target rotation: " + targetRotation);
    } else {
      myMove();
    }
  }
  
  private void handleTurnLeftTask() {
    switch (turnLeftSubState) {
      case TURNLEFT_BACKWARD:
        // Phase 1: Reculer pour éviter l'obstacle
        if (backwardStepCount < 5) { // Nombre d'étapes de recul
          myMoveBack();
          backwardStepCount++;
          sendLogMessage("Turn maneuver: backing up step " + backwardStepCount);
        } else {
          turnLeftSubState = TURNLEFT_ROTATE;
          sendLogMessage("Turn maneuver: starting rotation");
        }
        break;
        
      case TURNLEFT_ROTATE:
        // Phase 2: Tourner pour faire face à la direction opposée
        if (!isFacingDirection(targetRotation)) {
          // Déterminer le sens de rotation optimal (gauche ou droite)
          double angleDiff = normalizeAngleDifference(targetRotation - getHeading());
          if (Math.abs(angleDiff) < Math.PI) {
            // Chemin le plus court est dans le sens direct
            stepTurn(angleDiff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
          } else {
            // Chemin le plus court est dans le sens inverse
            stepTurn(angleDiff > 0 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
          }
          sendLogMessage("Turn maneuver: rotating, current heading: " + getHeading() + 
                        ", target: " + targetRotation + ", diff: " + angleDiff);
        } else {
          turnLeftSubState = TURNLEFT_COMPLETE;
          sendLogMessage("Turn maneuver: rotation complete");
        }
        break;
        
      case TURNLEFT_COMPLETE:
        // Phase 3: Retour à la formation triangulaire
        state = TRIANGLE_FORMATION;
        sendLogMessage("Turn maneuver complete, resuming triangle formation");
        break;
    }
  }
  
  // --- MÉTHODES DE DÉTECTION ET DE RÉACTION ---
  private void handleRadarDetection() {
    freeze = false;
    friendlyFire = true;
    checkTeamCollision();
    
    // Actualiser le temps de scan
    lastScanTime = System.currentTimeMillis();
    
    // Réinitialiser la distance pour recalculer l'ennemi le plus proche
    double closestDistance = Double.MAX_VALUE;
    int closestEnemyId = -1;
    
    for (IRadarResult o : detectRadar()) {
      // Détecter les ennemis
      if (isEnemyBot(o.getObjectType())) {
        double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        // Générer un ID unique basé sur la position
        int enemyId = (int)(enemyX * 1000 + enemyY);
        
        // Mettre à jour le tracker d'ennemis
        if (enemyTracker.containsKey(enemyId)) {
          enemyTracker.get(enemyId).update(enemyX, enemyY, o.getObjectDistance());
        } else {
          enemyTracker.put(enemyId, new EnemyInfo(enemyX, enemyY, o.getObjectDistance(), o.getObjectType()));
        }
        
        // Vérifier si c'est l'ennemi le plus proche
        if (o.getObjectDistance() < closestDistance) {
          closestDistance = o.getObjectDistance();
          closestEnemyId = enemyId;
          
          // Stocker les coordonnées pour les autres robots
          lastEnemyX = enemyX;
          lastEnemyY = enemyY;
          
          // Envoyer un message FIRE à tous les robots de l'équipe
          sendMessage(FIRE, enemyX, enemyY, o.getObjectDistance());
        }
      }
      
      // Vérifier les obstacles latéraux
      if (o.getObjectDistance() <= 100 && 
          !isRoughlySameDirection(o.getObjectDirection(), getHeading()) && 
          o.getObjectType() != IRadarResult.Types.BULLET) {
        freeze = true;
      }
      
      // Vérifier le risque de tir ami
      if (isTeamOrWreck(o.getObjectType()) && fireOrder && onTheWay(o.getObjectDirection())) {
        friendlyFire = false;
      }
      
      // Vérifier les obstacles pour le DODGE
      if (state != DODGE && 
          ((o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 150) || 
          checkEnnemie())) {
        handleObstacleDetection(o);
      }
    }
    
    // Mettre à jour la cible actuelle si nous avons trouvé un ennemi plus proche
    if (closestEnemyId != -1) {
      targetEnemyId = closestEnemyId;
      // Mettre à jour les coordonnées cible pour le tir
      EnemyInfo target = enemyTracker.get(targetEnemyId);
      targetX = target.x;
      targetY = target.y;
      fireOrder = true;
    }
  }
  
  // Méthode pour supprimer les ennemis qui n'ont pas été détectés récemment
  private void cleanupStaleEnemies() {
    long currentTime = System.currentTimeMillis();
    enemyTracker.entrySet().removeIf(entry -> 
        currentTime - entry.getValue().lastSeen > ENEMY_TRACK_TIMEOUT);
    
    // Si la cible actuelle n'est plus dans le tracker, réinitialiser
    if (targetEnemyId != -1 && !enemyTracker.containsKey(targetEnemyId)) {
      targetEnemyId = -1;
      fireOrder = false;
    }
  }
  
  private boolean isEnemyBot(IRadarResult.Types type) {
    return type == IRadarResult.Types.OpponentMainBot || 
           type == IRadarResult.Types.OpponentSecondaryBot;
  }
  
  private boolean isTeamOrWreck(IRadarResult.Types type) {
    return type == IRadarResult.Types.TeamMainBot || 
           type == IRadarResult.Types.TeamSecondaryBot || 
           type == IRadarResult.Types.Wreck;
  }
  
  private void processEnemyDetection(IRadarResult o) {
    double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
    double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
    sendMessage(FIRE, enemyX, enemyY, o.getObjectDistance());
  }
  
  private void handleObstacleDetection(IRadarResult o) {
    state = DODGE;
    double obstacleX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
    double obstacleY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
    sendMessage(DODGE, obstacleX, obstacleY);
    sendLogMessage("DODGE state activated. Obstacle detected at (" + obstacleX + ", " + obstacleY + ")");
  }
  
  private boolean isObstacleAhead() {
    if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL)
      return true;
      
    for (IRadarResult o : detectRadar()) {
      if (isRoughlySameDirection(o.getObjectDirection(), getHeading()) &&
          o.getObjectDistance() <= 50) {
        return true;
      }
    }
    return false;
  }
  
  private void processFireOrders() {
    if (fireRythm == 0) {
      firePosition(targetX, targetY);
      fireRythm++;
      return;
    }
    fireRythm++;
    if (fireRythm == Parameters.bulletFiringLatency) fireRythm = 0;

    return;
  }
  
  // --- MÉTHODES DE DÉPLACEMENT ---
  private boolean moveToCoordinates(double targetX, double targetY) {
    double xDiff = targetX - myX;
    double yDiff = targetY - myY;
    
    if (checkTeamCollision()) return false;
  
    // Approche en deux phases : d'abord Y puis X, sauf s'il y a un obstacle
    if (Math.abs(yDiff) > 20) { 
      if (!isObstacleAhead(Parameters.SOUTH) && yDiff > 0) {
        return moveAlongAxis(Parameters.SOUTH);
      } else if (!isObstacleAhead(Parameters.NORTH) && yDiff < 0) {
        return moveAlongAxis(Parameters.NORTH);
      }
    }
    
    if (Math.abs(xDiff) > 20) {
      if (!isObstacleAhead(Parameters.EAST) && xDiff > 0) {
        return moveAlongAxis(Parameters.EAST);
      } else if (!isObstacleAhead(Parameters.WEST) && xDiff < 0) {
        return moveAlongAxis(Parameters.WEST);
      }
    }
    
    // Si les deux directions sont bloquées, essayer l'autre direction en premier
    if (Math.abs(xDiff) > 20) {
      if (!isObstacleAhead(Parameters.EAST) && xDiff > 0) {
        return moveAlongAxis(Parameters.EAST);
      } else if (!isObstacleAhead(Parameters.WEST) && xDiff < 0) {
        return moveAlongAxis(Parameters.WEST);
      }
    }
    
    if (Math.abs(yDiff) > 20) {
      if (!isObstacleAhead(Parameters.SOUTH) && yDiff > 0) {
        return moveAlongAxis(Parameters.SOUTH);
      } else if (!isObstacleAhead(Parameters.NORTH) && yDiff < 0) {
        return moveAlongAxis(Parameters.NORTH);
      }
    }
    
    return false;
  }
  
  private boolean isObstacleAhead(double direction) {
    for (IRadarResult o : detectRadar()) {
      if (isRoughlySameDirection(o.getObjectDirection(), direction) &&
          o.getObjectDistance() <= 50) {
        return true;
      }
    }
    return false;
  }
  
  private boolean moveAlongAxis(double targetDirection) {
    if (!isSameDirection(getHeading(), targetDirection)) {
      turnTowards(targetDirection);
      return false;
    } else {
      myMove();
      return true;
    }
  }
  
  private void turnTowards(double targetDirection) {
    double angleDiff = normalizeAngle(targetDirection - getHeading());
    stepTurn(angleDiff < Math.PI ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
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
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return Math.abs(angle);
  }
  
  // Normalise un angle de direction entre -PI et PI
  private double normalizeHeading(double angle) {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;  // Retourne l'angle signé, pas sa valeur absolue
  }
  
  // Calcule la différence d'angle normalisée entre deux directions
  private double normalizeAngleDifference(double diff) {
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    return diff;  // Retourne la différence signée, pas sa valeur absolue
  }
  
  private boolean isFacingDirection(double direction) {
    return Math.abs(normalizeAngleDifference(direction - getHeading())) < ANGLE_PRECISION;
  }
  
  private boolean isSameDirection(double dir1, double dir2) {
    return normalizeAngle(dir1 - dir2) < ANGLE_PRECISION;
  }
  
  private boolean isRoughlySameDirection(double dir1, double dir2) {
    return Math.abs(normalizeAngle(dir1) - normalizeAngle(dir2)) < FIRE_ANGLE_PRECISION;
  }
  
  private void firePosition(double x, double y) {
    double angle;
    if (myX <= x) {
      angle = Math.atan((y - myY) / (x - myX));
    } else {
      angle = Math.PI + Math.atan((y - myY) / (x - myX));
    }
    fire(angle);
  }
  
  private boolean onTheWay(double angle) {
    double targetAngle;
    if (myX <= targetX) {
      targetAngle = Math.atan((targetY - myY) / (targetX - myX));
    } else {
      targetAngle = Math.PI + Math.atan((targetY - myY) / (targetX - myX));
    }
    return isRoughlySameDirection(angle, targetAngle);
  }
  
  private int determineTeam() {
    return isSameDirection(getHeading(), Parameters.EAST) ? TEAM_A : TEAM_B;
  }
  
  // --- MÉTHODES DE FORMATION ---
  private double[] calculateFormationPosition(double centerX, double centerY, int robotId) {
    double[] position = new double[2];
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
    
    // Garder les positions à l'intérieur des limites de l'arène
    position[0] = Math.min(Math.max(position[0], SAFETY_MARGIN), ARENA_WIDTH - SAFETY_MARGIN);
    position[1] = Math.min(Math.max(position[1], SAFETY_MARGIN), ARENA_HEIGHT - SAFETY_MARGIN);
    
    return position;
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
        processFireMessage(parts);
        break;
      case ROGER:
        processRogerMessage(parts);
        break;
      case DODGE:
        processDodgeMessage(parts);
        break;
      default:
        sendLogMessage("Unknown message type: " + messageType);
        break;
    }
  }
  
  private void processFireMessage(String[] parts) {
    fireOrder = true;
    countDown = 0;
    targetX = Double.parseDouble(parts[3]);
    targetY = Double.parseDouble(parts[4]);
    
    // Si le message contient une distance
    double distance = Double.MAX_VALUE;
    if (parts.length > 5) {
      try {
        distance = Double.parseDouble(parts[5]);
      } catch (NumberFormatException e) {
        // Ignorer si la conversion échoue
      }
    }
    
    // Mettre à jour le tracker d'ennemis
    int enemyId = (int)(targetX * 1000 + targetY);
    
    // N'accepter que si l'ennemi est assez proche ou si nous n'avons pas de cible
    if ((distance < MAX_FIRING_DISTANCE || targetEnemyId == -1) && 
        (targetEnemyId == -1 || distance < enemyTracker.getOrDefault(targetEnemyId, new EnemyInfo(0, 0, Double.MAX_VALUE, null)).distance)) {
      
      if (!enemyTracker.containsKey(enemyId)) {
        enemyTracker.put(enemyId, new EnemyInfo(targetX, targetY, distance, null));
      } else {
        enemyTracker.get(enemyId).update(targetX, targetY, distance);
      }
      
      targetEnemyId = enemyId;
    }
  }
  
  private void processRogerMessage(String[] parts) {
    int whoAreYou = Integer.parseInt(parts[0]);
    boolean ready = Boolean.parseBoolean(parts[3]);
    if (whoAreYou == ALPHA) readyAlpha = ready;
    if (whoAreYou == BETA) readyBeta = ready;
    if (whoAreYou == GAMMA) readyGamma = ready;
  }
  
  private void processDodgeMessage(String[] parts) {
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
    
    double obstacleY = obstacleYtmp < betaY ? obstacleYtmp + (TRIANGLE_SIDE * 2) : obstacleYtmp - (TRIANGLE_SIDE * 2);
    double[] pos = calculateFormationPosition(obstacleX, obstacleY, whoAmI);
    formationX = pos[0];
    formationY = pos[1];
    state = DODGE;
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
      if ((o.getObjectType() == IRadarResult.Types.TeamMainBot ||
          o.getObjectType() == IRadarResult.Types.TeamSecondaryBot) &&
          o.getObjectDistance() <= 150 && onTheWay(Math.PI)) {
        return true;
      }
    }
    return false;
  }
  
  // --- MÉTHODES D'ODOMÉTRIE ET DE DÉTECTION DE COLLISION ---
  private void updateOdometry() {
    boolean collision = detectCollision();
    if (isMoving) {
      updatePosition(Parameters.teamAMainBotSpeed, collision);
      isMoving = false;
    } else if (isMovingBack) {
      updatePosition(-Parameters.teamAMainBotSpeed, collision);
      isMovingBack = false;
    }
  }
  
  private void updatePosition(double speed, boolean collision) {
    if (!collision) {
      myX += speed * Math.cos(getHeading());
      myY += speed * Math.sin(getHeading());
    } else if (speed < 0) {
      sendLogMessage("Collision detected during backward movement!");
    }
  }
  
  private boolean detectCollision() {    
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectDistance() < COLLISION_THRESHOLD) {
        return true;
      }
    }
    return false;
  }
  
  private void refinePositionWithRadar() {
    // Méthode pour améliorer la précision de la position (à développer si nécessaire)
  }
  
  // --- MÉTHODES DE DÉBOGAGE ---
  private void logDebugMessages() {
    boolean debug = true;
    if (!debug || state == SINK) return;
    
    String prefix = "";
    if (whoAmI == ALPHA) prefix = "ALPHA";
    else if (whoAmI == BETA) prefix = "#BETA";
    else if (whoAmI == GAMMA) prefix = "#GAMMA";
    
    sendLogMessage(prefix + "State=" + state + 
                  " current=(" + (int) myX + ", " + (int) myY + 
                  ") target=(" + (int) formationX + ", " + (int) formationY + ")");
  }

  /**
   * Vérifie si un ennemi est détecté par le radar.
   * @return true si un ennemi est détecté, false sinon.
   */
  private boolean checkEnnemie() {
    for (IRadarResult o : detectRadar()) {
      if (isEnemyBot(o.getObjectType())) {
        double distance = o.getObjectDistance();
        if (distance < ENEMY_DETECTION_THRESHOLD) {
          lastEnemyX = myX + distance * Math.cos(o.getObjectDirection());
          lastEnemyY = myY + distance * Math.sin(o.getObjectDirection());
          
          // Ajouter ou mettre à jour dans le tracker d'ennemis
          int enemyId = (int)(lastEnemyX * 1000 + lastEnemyY);
          if (!enemyTracker.containsKey(enemyId)) {
            enemyTracker.put(enemyId, new EnemyInfo(lastEnemyX, lastEnemyY, distance, o.getObjectType()));
          } else {
            enemyTracker.get(enemyId).update(lastEnemyX, lastEnemyY, distance);
          }
          
          sendMessage(FIRE, lastEnemyX, lastEnemyY, distance);
          return true;
        }
      }
    }
    return false;
  }
}
