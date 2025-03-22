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
  private static final double ANGLE_PRECISION = 0.03; 
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
  private static final double TRIANGLE_SIDE = 300;
  private static final double COLLISION_THRESHOLD = 120;
  private static final double ARENA_WIDTH = 3000;
  private static final double ARENA_HEIGHT = 2000;
  private static final double SAFETY_MARGIN = 100.0;
private static final double ENEMY_DETECTION_THRESHOLD = 135;
  
  // Paramètres de tir améliorés
  private static final int FIRE_COOLDOWN = Parameters.bulletFiringLatency - 2; // Tire plus fréquemment
  private static final double MAX_FIRING_DISTANCE = 1000.0; // Distance max de tir efficace
  private static final long ENEMY_TRACK_TIMEOUT = 3000; // Temps en ms pour oublier un ennemi non détecté

  // Paramètres du filtre de Kalman pour l'odométrie
  private static final double SENSOR_NOISE = 2.0; // Réduit de 5.0 à 2.0 pour moins filtrer les mesures
  private static final double PROCESS_NOISE = 3.0; // Augmenté de 2.0 à 3.0 pour permettre plus de dynamisme

  private static final int GRID_SIZE = 15; // Taille de la grille pour la planification
  private static final double GRID_RESOLUTION = 200; // Taille d'une cellule en unités

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
  private boolean alive;
  
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
  
  // Variables pour l'estimation de position avec Kalman
  private double estimatedX, estimatedY;
  private double uncertaintyX = 1.0, uncertaintyY = 1.0;
  private double velocityX = 0.0, velocityY = 0.0; // Estimation de la vitesse
  private double lastUpdateTime = 0;
  
  // Add this at the class level
  private double lastTargetX = -1, lastTargetY = -1;
  private static final double TARGET_CHANGE_THRESHOLD = 50.0;

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
    // If needed, invert Y component for prediction
    public double[] predictPosition(int timeSteps) {
      double dx = x - lastX;
      double dy = y - lastY;
      
      return new double[] {
        x + dx * timeSteps,
        y + dy * timeSteps  // No change needed
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
    alive = true;
    targetX = ARENA_WIDTH/2;
    targetY = ARENA_HEIGHT/2;
    countDown = 0;
    
    // Initialisation du filtre de Kalman
    estimatedX = myX;
    estimatedY = myY;
    velocityX = 0.0;
    velocityY = 0.0;
    lastUpdateTime = System.currentTimeMillis();
    
    // Définition de la position cible
    initializeFormationPosition();
  }
  
  private void initializeFormationPosition() {
    double centerX = Parameters.teamAMainBot2InitX + (myTeam == TEAM_A ? 200 : -200);
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
    // System.out.println(getWhoAmIString() + " state :" + state + " targetX: " + targetX + " targetY: " + targetY);
    // // Print radar information for debugging
    // for (IRadarResult o : detectRadar()) {
    //   System.out.println("Radar detected: Type=" + o.getObjectType() + 
    //              ", Distance=" + o.getObjectDistance() + 
    //              ", Direction=" + o.getObjectDirection());
    // }
    updateOdometry();
    alive = getHealth() > 1;
    // Vérification de la santé
    if (!alive) {
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
    
    // Only update target if it has changed significantly
    if (Math.hypot(formationX - lastTargetX, formationY - lastTargetY) > TARGET_CHANGE_THRESHOLD) {
      lastTargetX = formationX;
      lastTargetY = formationY;
      // System.out.println("Updated target to: (" + Math.round(formationX) + "," + Math.round(formationY) + ")");
    }
    
    // Rest of the method remains the same
    if (distToTarget < 30) {
      state = FINAL_ORIENTATION;
    } else {
      if(!checkTeamCollision())
        moveToCoordinates(lastTargetX, lastTargetY);
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
    boolean obstacleDetected = false;
    double dodgeX = myX;
    double dodgeY = myY;
  
    for (IRadarResult o : detectRadar()) {
      // System.out.println("Detected obstacle: Type=" + o.getObjectType() + ", Distance=" + o.getObjectDistance() + ", WhoAmI=" + getWhoAmIString());
      if (o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 300) {
        obstacleDetected = true;
        double obstacleX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double obstacleY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
  
        // Calculer une position d'esquive en diagonale
        targetX = myX + 500 ;
        targetY = myY > obstacleY ? myY + 300 : myY - 300;
        break;
      }
    }
      System.out.println(getWhoAmIString() + " j'envoie (" + dodgeX + "," + dodgeY +")");
    
    if (obstacleDetected) {
      moveToCoordinates(targetX, targetY);
    } else {
      // Revenir à la formation
      state = TRIANGLE_FORMATION;
    }
  }
  
  private void handleMoveTask() {
    if (isObstacleAhead() && alive) {
      state = TURNLEFTTASK;
      oldAngle = getHeading();
      // Initialisation du sous-état TURNLEFTTASK
      turnLeftSubState = TURNLEFT_BACKWARD;
      backwardStepCount = 0;
      // Définir l'angle cible (opposé à la direction actuelle)
      targetRotation = normalizeHeading(getHeading() + Math.PI);
      sendLogMessage("Obstacle ahead, initiating turn maneuver. Target rotation: " + targetRotation);
    } else {
      // System.out.println("Target coordinates: X=" + targetX + ", Y=" + targetY);
      double[] pos = calculateFormationPosition(targetX - 100, targetY, whoAmI);
      formationX = pos[0];
      formationY = pos[1];
      if (alive  && state != TRIANGLE_FORMATION && state != FINAL_ORIENTATION)
      state = DODGE;
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
        System.out.println("à ok");
        sendLogMessage("Turn maneuver complete, resuming triangle formation");
        break;
    }
  }
  
  // --- MÉTHODES DE DÉTECTION ET DE RÉACTION ---
  private void handleRadarDetection() {
    freeze = false;
    friendlyFire = true;    
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
          checkEnnemie()) && alive) {
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
  }
  
  private boolean isObstacleAhead() {
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
 /**
 * Déplace le robot vers (targetX, targetY) en évitant les obstacles.
 * Cette méthode vérifie d'abord si le chemin direct est dégagé.
 * Sinon, elle cherche une direction alternative en tournant par petits incréments.
 */
private void moveToCoordinates(double targetX, double targetY) {
  // System.out.println("Target coordinates: X=" + targetX + ", Y=" + targetY + ", WhoAmI: " + getWhoAmIString());
  double dx = targetX - myX;
  double dy = targetY - myY;
  double distance = Math.hypot(dx, dy);

  if (distance < 30) {
    return; // Already at destination
  }

  double angleToTarget = normalizeHeading(Math.atan2(dy, dx));
  // if (whoAmI = GAMMA){
  //   System.out.println(getWhoAmIString() +" angle " +  angleToTarget);
  // }
  
  if (isObstacleOnPath(angleToTarget)) {
    // Stratégie d'évitement d'obstacles - Recherche d'un angle libre
    boolean pathFound = false;
    double testAngle = angleToTarget;
    
    // Chercher un chemin libre en incrémentant l'angle
    for (int i = 1; i <= 12 && !pathFound; i++) {
      // Essayer alternativement à droite puis à gauche avec des incréments croissants
      double increment = (i % 2 == 1) ? (i * Math.PI / 12) : -(i * Math.PI / 12);
      testAngle = normalizeHeading(angleToTarget + increment);
      
      if (!isObstacleOnPath(testAngle)) {
        // Chemin trouvé, avancer dans cette direction
        pathFound = true;
        turnAndMove(testAngle);
        sendLogMessage("Obstacle avoided: moving at angle " + testAngle);
      }
    }
    
    if (!pathFound) {
      // Si aucun chemin n'est trouvé, reculer temporairement
      myMoveBack();
      sendLogMessage("All paths blocked, backing up");
    }
  } else {
    // Pas d'obstacle, utiliser turnAndMove directement
    turnAndMove(angleToTarget);
  }
}
    
private String getWhoAmIString() {
switch (whoAmI) {
  case ALPHA:
    return "I am ALPHA";
  case BETA:
    return "I am BETA";
  case GAMMA:
    return "I am GAMMA";
  default:
    return "Unknown identity";
}
}

private void turnAndMove(double desiredAngle) {
  double angleDiff = normalizeAngleDifference(desiredAngle - getHeading());
  
  if (Math.abs(angleDiff) > Math.PI/2) {
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
/**
* Vérifie si un obstacle se trouve sur le chemin dans la direction donnée.
* On parcourt les relevés du radar et si un obstacle se trouve dans un cône défini
* par 'tolerance' et à une distance critique, on considère le chemin bloqué.
*/
private boolean isObstacleOnPath(double angle) {
  for (IRadarResult o : detectRadar()) {
    if (o.getObjectType() == IRadarResult.Types.BULLET) continue;
    
    double tolerance = 0.3; // Augmenter la tolérance pour réduire la sensibilité
    double diff = normalizeAngleDifference(o.getObjectDirection() - angle);
    if (Math.abs(diff) < tolerance ) { // Réduire la distance critique
      return true;
    }
  }
  return false;
}

  private boolean isObstacleAhead(double direction) {
    for (IRadarResult o : detectRadar()) {
      if (isRoughlySameDirection(o.getObjectDirection(), direction) &&
          o.getObjectDistance() <= COLLISION_THRESHOLD*1.02) {
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
    double angleDiff = normalizeAngleDifference(targetDirection - getHeading());
    stepTurn(angleDiff > 0 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
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
    double newTargetX = Double.parseDouble(parts[3]);
    double newTargetY = Double.parseDouble(parts[4]);
    double newDistance = Math.hypot(newTargetX - myX, newTargetY - myY);
    
    // Mettre à jour uniquement si la nouvelle cible est plus proche
    if (newDistance < Math.hypot(targetX - myX, targetY - myY)) {
      targetX = newTargetX;
      targetY = newTargetY;
    }
    
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
        enemyTracker.put(enemyId, new EnemyInfo(targetX, targetY, distance, null)); // Added null as the fourth parameter
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
    double betaY = myY + (Math.random() * 100 - 50) ;
    
    if (parts.length >= 7) {
      betaX = Double.parseDouble(parts[5]);
      betaY = Double.parseDouble(parts[6]);
    } else if (whoAmI == BETA) {
      sendMessage(DODGE, obstacleX, obstacleYtmp, betaX, betaY);
    } else {
      return;
    }
    
    double obstacleY = obstacleYtmp < betaY ? obstacleYtmp + (500) : obstacleYtmp - (500);
    double[] pos = calculateFormationPosition(obstacleX, obstacleYtmp, whoAmI);
    formationX = pos[0];
    formationY = pos[1];
    if (alive)
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
          o.getObjectDistance() <= 200 && onTheWay(Math.PI)) {
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
    
    // Affiner la position avec les données du radar
    refinePositionWithRadar();
  }
  
  private void updatePosition(double speed, boolean collision) {
    if (!collision) {
      // Calculer le temps écoulé depuis la dernière mise à jour
      long currentTime = System.currentTimeMillis();
      double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convertir en secondes
      if (deltaTime <= 0) deltaTime = 0.01; // Éviter la division par zéro
      lastUpdateTime = currentTime;
      
      // Calculer la nouvelle position basée sur la direction et la vitesse
      double newX = myX + speed * Math.cos(getHeading());
      double newY = myY + speed * Math.sin(getHeading());
      
      // Calculer la vitesse instantanée
      double instantVelocityX = (newX - myX) / deltaTime;
      double instantVelocityY = (newY - myY) / deltaTime;
      
      // Mettre à jour la vitesse estimée avec un facteur de lissage
      double alpha = 0.3; // Facteur de lissage pour la vitesse (0.3 = 30% nouvelle mesure + 70% estimation précédente)
      velocityX = alpha * instantVelocityX + (1 - alpha) * velocityX;
      velocityY = alpha * instantVelocityY + (1 - alpha) * velocityY;
      
      // Prédire la position avec le modèle de vitesse
      double predictedX = estimatedX + velocityX * deltaTime;
      double predictedY = estimatedY + velocityY * deltaTime;
      
      // Mettre à jour avec le filtre de Kalman, en donnant plus de poids à la mesure directe
      double measurementWeight = 0.7; // Augmenter l'influence de la mesure directe
      estimatedX = kalmanFilter(predictedX, newX, uncertaintyX, measurementWeight);
      estimatedY = kalmanFilter(predictedY, newY, uncertaintyY, measurementWeight);
      
      // Utiliser l'estimation comme position actuelle
      myX = estimatedX;
      myY = estimatedY;
      
      // Debug - afficher les vitesses
      if (speed != 0) {
        sendLogMessage("Vitesse: vX=" + Math.round(velocityX) + ", vY=" + Math.round(velocityY));
      }
    } else if (speed < 0) {
      sendLogMessage("Collision détectée - position non mise à jour");
      // En cas de collision, réinitialiser la vitesse estimée
      velocityX = 0;
      velocityY = 0;
    }
  }
  
  /**
   * Implémentation du filtre de Kalman avec pondération ajustable.
   * @param predicted Valeur prédite par le modèle de vitesse
   * @param measured Valeur mesurée actuelle
   * @param uncertainty Incertitude actuelle sur l'estimation
   * @param measurementWeight Poids donné à la mesure (0-1)
   * @return Nouvelle estimation filtrée
   */
  private double kalmanFilter(double predicted, double measured, double uncertainty, double measurementWeight) {
    // Calculer le gain de Kalman standard
    double kalmanGain = uncertainty / (uncertainty + SENSOR_NOISE);
    
    // Ajuster le gain avec le poids spécifié pour la mesure
    kalmanGain = (1 - measurementWeight) * kalmanGain + measurementWeight;
    
    // Mise à jour de l'estimation
    double newEstimated = predicted + kalmanGain * (measured - predicted);
    
    // Mise à jour de l'incertitude
    double newUncertainty = (1 - kalmanGain) * uncertainty + PROCESS_NOISE;
    
    // Mettre à jour la variable d'incertitude correspondante
    if (predicted == estimatedX) {
      uncertaintyX = newUncertainty;
    } else {
      uncertaintyY = newUncertainty;
    }
    
    return newEstimated;
  }
  
  private void refinePositionWithRadar() {
    // Utiliser des repères fixes détectés par le radar pour améliorer l'estimation de position
    for (IRadarResult o : detectRadar()) {
      // Si on détecte un obstacle fixe (wreck) à une distance raisonnable
      if (o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 400) {
        // Calculer la position absolue de l'obstacle
        double wreckX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double wreckY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        // Position estimée du robot basée sur l'obstacle fixe
        double robotX = wreckX - o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double robotY = wreckY - o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        // Utiliser cette information pour ajuster notre estimation de position
        // Donner moins de poids aux corrections basées sur le radar (0.3 = 30%)
        estimatedX = kalmanFilter(estimatedX, robotX, uncertaintyX * 1.5, 0.3);
        estimatedY = kalmanFilter(estimatedY, robotY, uncertaintyY * 1.5, 0.3);
        
        // Mettre à jour notre position estimée
        myX = estimatedX;
        myY = estimatedY;
        break; // Une seule correction par cycle pour éviter des ajustements trop brutaux
      }
    }
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
                  "target=(" + (int) formationX + ", " + (int) formationY + ")");
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
  
  private boolean detectCollision() {    
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectDistance() < COLLISION_THRESHOLD) {
        return true;
      }
    }
    return false;
  }

  // Méthode de création de grille d'occupation
  private boolean[][] createOccupancyGrid() {
    boolean[][] grid = new boolean[GRID_SIZE][GRID_SIZE];
    
    // Calculer les limites de la grille centrée sur le robot
    double gridStartX = myX - (GRID_SIZE/2) * GRID_RESOLUTION;
    double gridStartY = myY - (GRID_SIZE/2) * GRID_RESOLUTION;
    
    // Marquer les cellules occupées par des obstacles
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == IRadarResult.Types.BULLET) continue;
      
      double obstacleX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
      double obstacleY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
      
      // Convertir en indices de grille
      int gridX = (int)((obstacleX - gridStartX) / GRID_RESOLUTION);
      int gridY = (int)((obstacleY - gridStartY) / GRID_RESOLUTION);
      
      // Vérifier que les indices sont dans les limites
      if (gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE) {
        // Marquer les cellules voisines aussi (pour tenir compte de la taille des obstacles)
        for (int dx = -1; dx <= 1; dx++) {
          for (int dy = -1; dy <= 1; dy++) {
            int x = gridX + dx;
            int y = gridY + dy;
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
              grid[x][y] = true; // Cellule occupée
            }
          }
        }
      }
    }
    
    return grid;
  }
}
