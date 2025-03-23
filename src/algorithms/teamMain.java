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
  private static final int TARGET_CONSENSUS = 0xC105E; // Nouveau type de message pour le consensus de cible

  // États de tâches
  private static final int SINK = 0xBADC0DE1;
  private static final int MOVETASK = 2;
  private static final int TURNLEFTTASK = 3;
  private static final int TRIANGLE_FORMATION = 4;
  private static final int FINAL_ORIENTATION = 7;
  private static final int HUNT_MODE = 8; // Nouvel état pour la chasse aux ennemis
  
  // Sous-états pour TURNLEFTTASK
  private static final int TURNLEFT_BACKWARD = 0;
  private static final int TURNLEFT_ROTATE = 1;
  private static final int TURNLEFT_COMPLETE = 2;

  // Équipes
  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  // Paramètres de formation et carte
  private static final double TRIANGLE_SIDE = 300;
  private static final double COLLISION_THRESHOLD = 130;
  private static final double ARENA_WIDTH = 3000;
  private static final double ARENA_HEIGHT = 2000;
  private static final double SAFETY_MARGIN = 100.0;
private static final double ENEMY_DETECTION_THRESHOLD = 135;
  
  // Paramètres de tir améliorés
  private static final int FIRE_COOLDOWN = 3; // Réduit de 5 à 3 pour tirer plus rapidement
  private static final double MAX_FIRING_DISTANCE = 1200.0; // Distance max de tir augmentée
  private static final double OPTIMAL_FIRING_DISTANCE = 700.0; // Distance optimale pour le tir
  private static final double IMMEDIATE_FIRING_THRESHOLD = 400.0; // Tir immédiat si ennemi très proche
  private static final long ENEMY_TRACK_TIMEOUT = 3000; // Temps en ms pour oublier un ennemi non détecté

  // Paramètres du filtre de Kalman pour l'odométrie
  private static final double SENSOR_NOISE = 2.0; // Réduit de 5.0 à 2.0 pour moins filtrer les mesures
  private static final double PROCESS_NOISE = 3.0; // Augmenté de 2.0 à 3.0 pour permettre plus de dynamisme

  // Ajouter ces constantes et variables à votre classe
  private static final long TARGET_MAX_FOCUS_TIME = 8000; // 8 secondes max sur la même cible
  private long currentTargetStartTime = 0;

  // --- VARIABLES ---r 
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

  // Add these variables to your class
  private int consecutiveCollisionSteps = 0;
  private boolean tryingMoveBack = true; // Tracks which strategy we're using

  // État de vie des robots de l'équipe
  private boolean isAlphaAlive = true;
  private boolean isBetaAlive = true;
  private boolean isGammaAlive = true;
  private boolean teamMemberDied = false; // Indique si un membre de l'équipe est mort

  // Ajouter au niveau de la classe:
  private int nextEnemyId = 1;
  private Map<Integer, double[]> enemyPositions = new HashMap<>();

  private int persistentTargetId = -1;
  private long lastTargetUpdateTime = 0;
  private static final long TARGET_PERSISTENCE_TIME = 5000; // 5 secondes

  // Variables pour le consensus de cible
  private int teamConsensusTargetId = -1;
  private Map<Integer, Integer> targetVotes = new HashMap<>(); // Comptage des votes pour chaque cible
  private long lastTargetBroadcastTime = 0;
  private static final long TARGET_BROADCAST_INTERVAL = 500; // Intervalle de diffusion des préférences de cible en ms

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
      // Calculer le vecteur de déplacement
      double dx = x - lastX;
      double dy = y - lastY;
      
      // Calculer le temps écoulé depuis la dernière mise à jour
      long deltaTime = System.currentTimeMillis() - lastSeen;
      long previousDeltaTime = 100; // estimation raisonnable par défaut
      
      // Extrapoler la distance parcourue en fonction du temps
      double timeRatio = Math.min(2.0, Math.max(0.1, 100.0 / Math.max(1, deltaTime)));
      
      // Position prédite ajustée par le facteur temps
      double predictedX = x + (dx * timeSteps * timeRatio);
      double predictedY = y + (dy * timeSteps * timeRatio);
      
      // Limiter la prédiction à l'arène
      predictedX = Math.min(Math.max(predictedX, 50), ARENA_WIDTH - 50);
      predictedY = Math.min(Math.max(predictedY, 50), ARENA_HEIGHT - 50);
      
      return new double[] { predictedX, predictedY };
    }
  }

  // --- INITIALISATION ---
  public void activate() {
    determineInitialPosition();
    myTeam = determineTeam();
    
    if (whoAmI == GAMMA) {
      myX = (myTeam == TEAM_A) ? Parameters.teamAMainBot1InitX : Parameters.teamBMainBot1InitX;
      myY = (myTeam == TEAM_A) ? Parameters.teamAMainBot1InitY : Parameters.teamBMainBot1InitY;
    } else if (whoAmI == BETA) {
      myX = (myTeam == TEAM_A) ? Parameters.teamAMainBot2InitX : Parameters.teamBMainBot2InitX;
      myY = (myTeam == TEAM_A) ? Parameters.teamAMainBot2InitY : Parameters.teamBMainBot2InitY;
    } else if (whoAmI == ALPHA) {
      myX = (myTeam == TEAM_A) ? Parameters.teamAMainBot3InitX : Parameters.teamBMainBot3InitX;
      myY = (myTeam == TEAM_A) ? Parameters.teamAMainBot3InitY : Parameters.teamBMainBot3InitY;
    }

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

    // Initialiser les variables d'état de vie
    isAlphaAlive = true;
    isBetaAlive = true;
    isGammaAlive = true;
    teamMemberDied = false;
  }
  
  private void initializeFormationPosition() {
    double centerX = (myTeam == TEAM_A) ? Parameters.teamAMainBot2InitX + 100 : Parameters.teamBMainBot2InitX - 100;
    double centerY = (myTeam == TEAM_A) ? Parameters.teamAMainBot2InitY : Parameters.teamBMainBot2InitY;
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
 
  
  }
  
  // --- BOUCLE PRINCIPALE ---
  public void step() {
    // System.out.println(getWhoAmIString() + " state :" + state + " targetX: " + targetX + " targetY: " + targetY);
    // Print radar information for debugging
    // for (IRadarResult o : detectRadar()) {
    //   System.out.println("Radar detected: Type=" + o.getObjectType() + 
    //              ", Distance=" + o.getObjectDistance() + 
    //              ", Direction=" + o.getObjectDirection());
    // }
    // Print the list of enemies being tracked
    updateOdometry();
   
    // Vérification de la santé
    if (getHealth() < 1) {
      sendMessage(ROGER, false);

    }
    // Passer en mode chasse si un membre de l'équipe est mort et qu'on n'est pas déjà en chasse
    if (teamMemberDied && state != HUNT_MODE && state != SINK && targetEnemyId != -1) {
      sendLogMessage("Team member died, switching to hunt mode");
      state = HUNT_MODE;
    }

    logDebugMessages();
    processIncomingMessages();
    
    // Gestion de la détection et du tir
    handleRadarDetection();

    // Gérer le cooldown du tir 

    fireRythm++;
    if (fireRythm >= FIRE_COOLDOWN) fireRythm = 0;
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
      case HUNT_MODE:
        handleHuntMode();
        break;
        
    }
    
        // Logique de tir améliorée
        // Dans la méthode step(), remplacer le bloc de tir par:
        if (fireAtNearbyEnemy()) {
          // Si un tir a été effectué sur un ennemi proche, ne pas exécuter la logique de tir standard
        } else if (fireRythm == 0 && targetEnemyId != -1 && friendlyFire) {
          EnemyInfo target = enemyTracker.get(targetEnemyId);
      
          if (target != null && target.distance < MAX_FIRING_DISTANCE) {
              // Utiliser la prédiction de position pour augmenter la précision
              double[] predictedPos = target.predictPosition(2);
              
              // Vérifier s'il y a des obstacles avant de tirer
              if (isLineOfFireClear(predictedPos[0], predictedPos[1])) {
                  firePosition(predictedPos[0], predictedPos[1]);
              } else {
                  // Si ligne de tir bloquée et nous sommes en mode chasse,
                  // chercher une meilleure position de tir
                  if (state == HUNT_MODE) {
                      sendLogMessage("Ligne de tir bloquée - repositionnement");
                  }
              }
          }else{
              // Si la cible est hors de portée, arrêter de tirer
              if (isLineOfFireClear(myX + 1000 * Math.cos(getHeading()), 
              myY + 1000 * Math.sin(getHeading()))) {
fire(getHeading());
}
          }
      } else if (fireRythm == 0 && friendlyFire) {
          // Tir aléatoire - vérifier qu'il n'y a pas d'obstacle directement devant
          if (isLineOfFireClear(myX + 1000 * Math.cos(getHeading()), 
                              myY + 1000 * Math.sin(getHeading()))) {
              fire(getHeading());
          }
      }
        
    // Nettoyer les ennemis qui n'ont pas été détectés pendant un certain temps
    cleanupStaleEnemies();
    // Appeler selectTarget pour mettre à jour la cible périodiquement
    selectTarget();
    // Mise à jour de la cible consensuelle si nécessaire
    updateTeamConsensusTarget();
  }
  
  // --- HANDLERS POUR LES DIFFÉRENTS ÉTATS ---
  private void handleTriangleFormation() {
    double distToTarget = Math.hypot(myX - formationX, myY - formationY);
    if (distToTarget < 30 || ( distToTarget < 100 && (checkEnnemie() || isWreckNearby()))) {
      state = FINAL_ORIENTATION;
    } else {
      if(!checkTeamCollision())
        moveToCoordinates(formationX, formationY);
    }
  }

      // Rest of the method remains the same
      private boolean isWreckNearby() {
        for (IRadarResult o : detectRadar()) {
          if (o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 200) {
            return true;
          }
        }
        return false;
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
      if ((o.getObjectType() == IRadarResult.Types.Wreck || isEnemyBot(o.getObjectType())) && o.getObjectDistance() < 300) {
        obstacleDetected = true;
        double obstacleX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double obstacleY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
  
        // Calculer une position d'esquive en diagonale
        if (Math.hypot(obstacleX - targetX, obstacleY - targetY) < COLLISION_THRESHOLD * 2) {
          // Si l'obstacle est sur la cible, déplacer la cible
          targetX = targetX + 500;
          targetY = targetX > obstacleY ? targetX + 300 : targetX - 300;
        } 
        break;
      }
    }    
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
  
  // --- NOUVELLE MÉTHODE POUR GÉRER LE MODE CHASSE ---
  private void handleHuntMode() {
    // Vérification de cible existante (code déjà présent)
    if (targetEnemyId == -1) {
        selectTarget();
        if (targetEnemyId == -1) {
            state = TRIANGLE_FORMATION;
            return;
        }
    }
    
    // Récupération des infos sur la cible (code déjà présent)
    EnemyInfo target = enemyTracker.get(targetEnemyId);
    if (target == null) {
        selectTarget();
        if (targetEnemyId == -1) {
            state = TRIANGLE_FORMATION;
            return;
        }
        target = enemyTracker.get(targetEnemyId);
    }
    
    // NOUVEAU CODE: Logique d'interception
    double[] interceptPoint = calculateInterceptionPoint(target);
    double[] flankPoint = calculateFlankingPosition(interceptPoint[0], interceptPoint[1]);
    double interceptX = flankPoint[0];
    double interceptY = flankPoint[1];
    
    // Déplacement vers le point d'interception
    double distance = Math.hypot(interceptX - myX, interceptY - myY);
    
    if (distance < 30) {
        // Si on est suffisamment proche, on s'arrête et on tire
        fireRythm = 0; // Pour permettre de tirer immédiatement
    } else {
        // Si on est loin, on se déplace vers le point d'interception
        moveToCoordinatesAggressive(interceptX, interceptY);
    }
    
    // Envoyer des informations de débogage
    sendLogMessage("Hunting: target=" + targetEnemyId + 
                  ", distance=" + (int)distance + 
                  ", intercepting at (" + (int)interceptX + "," + (int)interceptY + ")");
}
  
  // --- MÉTHODES DE DÉTECTION ET DE RÉACTION ---
  private void handleRadarDetection() {
    friendlyFire = true;    
    // Actualiser le temps de scan
    lastScanTime = System.currentTimeMillis();
    
    // Réinitialiser les distances pour les différents types d'ennemis
    double closestMainBotDistance = Double.MAX_VALUE;
    double closestSecondaryBotDistance = Double.MAX_VALUE;
    int closestMainBotId = -1;
    int closestSecondaryBotId = -1;
    
    for (IRadarResult o : detectRadar()) {
      // Détecter les ennemis
      if (isEnemyBot(o.getObjectType())) {
        double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        // Générer un ID unique basé sur la position
        int enemyId = getOrCreateEnemyId(enemyX, enemyY);
        
        // Mettre à jour le tracker d'ennemis
        if (enemyTracker.containsKey(enemyId)) {
          enemyTracker.get(enemyId).update(enemyX, enemyY, o.getObjectDistance());
          // Make sure to update the type information
          enemyTracker.get(enemyId).type = o.getObjectType();
        } else {
          enemyTracker.put(enemyId, new EnemyInfo(enemyX, enemyY, o.getObjectDistance(), o.getObjectType()));
        }
        
        // Distinguer entre MainBot et SecondaryBot et garder le plus proche de chaque type
        if (o.getObjectType() == IRadarResult.Types.OpponentMainBot) {
          if (o.getObjectDistance() < closestMainBotDistance) {
            closestMainBotDistance = o.getObjectDistance();
            closestMainBotId = enemyId;
            
            // Stocker les coordonnées pour les autres robots
            lastEnemyX = enemyX;
            lastEnemyY = enemyY;
            
            // Include type code in the message (1 for MainBot)
            sendMessage(FIRE, enemyX, enemyY, o.getObjectDistance(), 1);
          }
        } else if (o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
          if (o.getObjectDistance() < closestSecondaryBotDistance) {
            closestSecondaryBotDistance = o.getObjectDistance();
            closestSecondaryBotId = enemyId;
            
            // Stocker les coordonnées uniquement si aucun MainBot n'est détecté
            if (closestMainBotId == -1) {
              lastEnemyX = enemyX;
              lastEnemyY = enemyY;
              
              // Include type code in the message (2 for SecondaryBot)
              sendMessage(FIRE, enemyX, enemyY, o.getObjectDistance(), 2);
            }
          }
        }
      }
      
      // Vérifier le risque de tir ami
      if ((o.getObjectType() == IRadarResult.Types.TeamMainBot || 
      o.getObjectType() == IRadarResult.Types.TeamSecondaryBot) && onTheWay(o.getObjectDirection())) {
        friendlyFire = false;
      }
      
      // Vérifier les obstacles pour le DODGE
      if (state != DODGE && 
          ((o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 150) || 
          checkEnnemie()) && alive) {
        handleObstacleDetection(o);
      }
    }
    
    // Prioriser les MainBots sur les SecondaryBots
    if (closestMainBotId != -1) {
      targetEnemyId = closestMainBotId;
      EnemyInfo target = enemyTracker.get(targetEnemyId);
      targetX = target.x;
      targetY = target.y;
    } else if (closestSecondaryBotId != -1) {
      targetEnemyId = closestSecondaryBotId;
      EnemyInfo target = enemyTracker.get(targetEnemyId);
      targetX = target.x; 
      targetY = target.y;
    }
  }
  
  // Méthode pour supprimer les ennemis qui n'ont pas été détectés récemment
  private void cleanupStaleEnemies() {
    long currentTime = System.currentTimeMillis();
    enemyTracker.entrySet().removeIf(entry -> 
        currentTime - entry.getValue().lastSeen > ENEMY_TRACK_TIMEOUT);

    // Si la cible actuelle n'est plus dans le tracker, réinitialiser
    // if (whoAmI == GAMMA) {
    //   for (Map.Entry<Integer, EnemyInfo> entry : enemyTracker.entrySet()) {
    //     EnemyInfo enemy = entry.getValue();
    //     System.out.println("Enemy ID: " + entry.getKey() + 
    //                        ", Position: (" + enemy.x + ", " + enemy.y + 
    //                        "), Last Seen: " + enemy.lastSeen + 
    //                        ", Distance: " + enemy.distance);
    //   }
    // }
    if (targetEnemyId != -1 && !enemyTracker.containsKey(targetEnemyId)) {
      targetEnemyId = -1;
      fireOrder = false;
    }
  }
  
  private boolean isEnemyBot(IRadarResult.Types type) {
    return type == IRadarResult.Types.OpponentMainBot || 
           type == IRadarResult.Types.OpponentSecondaryBot;
  }
  

  
  private void processEnemyDetection(IRadarResult o) {
    double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
    double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
    
    // Add type code: 1 for MainBot, 2 for SecondaryBot
    int typeCode = 0;
    if (o.getObjectType() == IRadarResult.Types.OpponentMainBot) {
      typeCode = 1;
    } else if (o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
      typeCode = 2;
    }
    
    sendMessage(FIRE, enemyX, enemyY, o.getObjectDistance(), typeCode);
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

  // --- MÉTHODES DE DÉPLACEMENT ---
 /**
 * Déplace le robot vers (targetX, targetY) en évitant les obstacles.
 * Cette méthode vérifie d'abord si le chemin direct est dégagé.
 * Sinon, elle cherche une direction alternative en tournant par petits incréments.
 */
private void moveToCoordinates(double targetX, double targetY) {
    // Vérifier les collisions imminentes
    IRadarResult nearestObstacle = null;
    double minDistance = COLLISION_THRESHOLD * 1.2;
    
    for (IRadarResult o : detectRadar()) {
        if (o.getObjectType() != IRadarResult.Types.BULLET && 
            o.getObjectDistance() < minDistance) {
            minDistance = o.getObjectDistance();
            nearestObstacle = o;
        }
    }
    
    // Si obstacle imminent, effectuer une manœuvre d'évitement
    if (nearestObstacle != null) {
        double obstacleDirection = nearestObstacle.getObjectDirection();
        double avoidanceAngle = normalizeHeading(obstacleDirection + Math.PI);
        
        // Si l'obstacle est vraiment proche, reculer
        if (minDistance < COLLISION_THRESHOLD * 0.5) {
            if (isFacingDirection(obstacleDirection)) {
                myMoveBack();
            } else {
                turnTowards(avoidanceAngle);
            }
            return;
        }
        
        // Sinon, contourner l'obstacle
        double perpendicular = normalizeHeading(obstacleDirection + Math.PI/2);
        double targetAngle = Math.atan2(targetY - myY, targetX - myX);
        
        // Choisir la direction perpendiculaire qui rapproche le plus de la cible
        double option1 = perpendicular;
        double option2 = normalizeHeading(perpendicular + Math.PI);
        double diff1 = Math.abs(normalizeAngleDifference(option1 - targetAngle));
        double diff2 = Math.abs(normalizeAngleDifference(option2 - targetAngle));
        
        double bestAvoidanceAngle = (diff1 < diff2) ? option1 : option2;
        turnAndMove(bestAvoidanceAngle);
        return;
    }
    
    // Pas d'obstacle imminent, continuer vers la cible
    double dx = targetX - myX;
    double dy = targetY - myY;
    double distance = Math.hypot(dx, dy);
    
    if (distance < 30) {
        return; // Destination atteinte
    }
    
    double angleToTarget = normalizeHeading(Math.atan2(dy, dx));
    turnAndMove(angleToTarget);
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
        
        double diff = normalizeAngleDifference(o.getObjectDirection() - angle);
        
        // Ajuster la tolérance en fonction de la distance
        // Plus l'obstacle est proche, plus la tolérance est stricte
        double tolerance = 0.5 - (0.3 * Math.min(1.0, COLLISION_THRESHOLD / o.getObjectDistance()));
        
        if (Math.abs(diff) < tolerance && o.getObjectDistance() < COLLISION_THRESHOLD * 2) {
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
    while (angle > Math.PI)
      angle -= 2 * Math.PI;
    while (angle < -Math.PI)
      angle += 2 * Math.PI;
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
    // Vérifier si nous avons une cible consensuelle valide
    if (teamConsensusTargetId != -1 && enemyTracker.containsKey(teamConsensusTargetId)) {
      EnemyInfo consensusTarget = enemyTracker.get(teamConsensusTargetId);
      
      // Si la cible consensuelle est différente de la cible actuelle
      if (Math.hypot(consensusTarget.x - x, consensusTarget.y - y) > 50) {
        // Priorité à la cible consensuelle
        x = consensusTarget.x;
        y = consensusTarget.y;
        sendLogMessage("Tir sur cible consensuelle plutôt que cible individuelle");
      }
    }
    
    // Vérifier si la ligne de tir est dégagée
    if (!isLineOfFireClear(x, y)) {
        sendLogMessage("Tir bloqué par obstacle - tentative de tir alternatif");
        
        // Chercher un angle alternatif pour le tir
        double baseAngle = Math.atan2(y - myY, x - myX);
        boolean fireAttempted = false;
        
        // Essayer des angles légèrement décalés
        for (int i = 1; i <= 3 && !fireAttempted; i++) {
            // Essayer à gauche puis à droite
            double offsetAngle = i * 0.1; // 0.1, 0.2, 0.3 radians (environ 6, 12, 18 degrés)
            
            // Essayer à gauche
            double testAngle = normalizeHeading(baseAngle - offsetAngle);
            double testX = myX + 1000 * Math.cos(testAngle);
            double testY = myY + 1000 * Math.sin(testAngle);
            
            if (isLineOfFireClear(testX, testY)) {
                fire(testAngle);
                fireAttempted = true;
                sendLogMessage("Tir alternatif à gauche: " + offsetAngle + " radians");
                continue;
            }
            
            // Essayer à droite
            testAngle = normalizeHeading(baseAngle + offsetAngle);
            testX = myX + 1000 * Math.cos(testAngle);
            testY = myY + 1000 * Math.sin(testAngle);
            
            if (isLineOfFireClear(testX, testY)) {
                fire(testAngle);
                fireAttempted = true;
                sendLogMessage("Tir alternatif à droite: " + offsetAngle + " radians");
            }
        }
        
        // Si aucun angle alternatif ne fonctionne, ne pas tirer
        if (!fireAttempted) {
            sendLogMessage("Impossible de trouver une ligne de tir dégagée");
            return;
        }
    } else {
        // Ligne de tir dégagée, tirer normalement
        double angle;
        if (myX <= x) {
            angle = Math.atan((y - myY) / (x - myX));
        } else {
            angle = Math.PI + Math.atan((y - myY) / (x - myX));
        }
        fire(angle);
    }
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
    if (isSameDirection(getHeading(), Parameters.EAST)) {
      return TEAM_A;
    }
    return TEAM_B;
  }
  
  // --- MÉTHODES DE FORMATION ---
  private double[] calculateFormationPosition(double centerX, double centerY, int robotId) {
    double[] position = new double[2];
    if (myTeam == TEAM_A) {
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
      case TARGET_CONSENSUS:
        processTargetConsensusMessage(parts);
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
    double distance = Double.MAX_VALUE;
    IRadarResult.Types enemyType = null;

    // Extract distance if available
    if (parts.length > 5) {
      try {
        distance = Double.parseDouble(parts[5]);
      } catch (NumberFormatException e) {
        // Ignore if conversion fails
      }
    }
    
    // Extract enemy type if available (starting at index 6)
    if (parts.length > 6) {
      try {
        int typeCode = Integer.parseInt(parts[6]);
        if (typeCode == 1) {
          enemyType = IRadarResult.Types.OpponentMainBot;
        } else if (typeCode == 2) {
          enemyType = IRadarResult.Types.OpponentSecondaryBot;
        }
      } catch (NumberFormatException e) {
        // Ignore if conversion fails
      }
    }
    
    // Generate enemy ID and update tracker
    int enemyId = getOrCreateEnemyId(newTargetX, newTargetY);
    
    // Check if we already have this enemy in our tracker
    boolean isNewEnemy = !enemyTracker.containsKey(enemyId);
    
    // If it's a new enemy or an update to an existing one
    if (isNewEnemy) {
      enemyTracker.put(enemyId, new EnemyInfo(newTargetX, newTargetY, distance, enemyType));
    } else {
      // Update existing enemy
      EnemyInfo existingEnemy = enemyTracker.get(enemyId);
      existingEnemy.update(newTargetX, newTargetY, distance);
      // Update type only if we didn't have it before or if we're getting a MainBot type
      if (existingEnemy.type == null || 
          (enemyType != null && enemyType == IRadarResult.Types.OpponentMainBot)) {
        existingEnemy.type = enemyType;
      }
    }
    
    // Decide whether to switch targets based on priority
    boolean shouldSwitchTarget = false;
    
    // If we don't have a target yet, take this one
    if (targetEnemyId == -1) {
      shouldSwitchTarget = true;
    } 
    // If this is a MainBot and our current target is not, switch
    else if (enemyType == IRadarResult.Types.OpponentMainBot && 
             enemyTracker.containsKey(targetEnemyId) &&
             enemyTracker.get(targetEnemyId).type != IRadarResult.Types.OpponentMainBot) {
      shouldSwitchTarget = true;
    }
    // If both are same type (or unknown), prefer the closer one
    else if ((enemyType == null || 
              enemyTracker.get(targetEnemyId).type == null ||
              enemyType == enemyTracker.get(targetEnemyId).type) &&
             distance < enemyTracker.get(targetEnemyId).distance) {
      shouldSwitchTarget = true;
    }
    
    if (shouldSwitchTarget) {
      targetEnemyId = enemyId;
      targetX = newTargetX;
      targetY = newTargetY;
    }
  }
  
  private void processRogerMessage(String[] parts) {
    System.out.println("Received ROGER message: " + parts);
    for (int i = 0; i < parts.length; i++) {
      System.out.println("Part " + i + ": " + parts[i]);
    }
    int whoAreYou = Integer.parseInt(parts[0]);
    boolean statusUpdate = parts.length > 3 ? Boolean.parseBoolean(parts[3]) : true;
    
    // Si le message indique que le robot est mort (statusUpdate = false)
    if (!statusUpdate) {
      if (whoAreYou == ALPHA) isAlphaAlive = false;
      if (whoAreYou == BETA) isBetaAlive = false;
      if (whoAreYou == GAMMA) isGammaAlive = false;
      
      teamMemberDied = true;
      sendLogMessage("Team member " + whoAreYou + " died. Switching to hunt mode.");
      return;
    }
    
    // Traitement habituel des messages ROGER pour la formation
    if (whoAreYou == ALPHA) readyAlpha = statusUpdate;
    if (whoAreYou == BETA) readyBeta = statusUpdate;
    if (whoAreYou == GAMMA) readyGamma = statusUpdate;
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
    
    // double obstacleY = obstacleYtmp < betaY ? obstacleYtmp + (500) : obstacleYtmp - (500);
    // double[] pos = calculateFormationPosition(obstacleX, obstacleYtmp, whoAmI);
    // formationX = pos[0];
    // formationY = pos[1];
    if (alive)
    state = DODGE;
  }
  
  private void processTargetConsensusMessage(String[] parts) {
    // Extraire l'ID de la cible proposée
    int proposedTargetId = Integer.parseInt(parts[3]);
    
    // Si cette cible existe dans notre tracker
    if (enemyTracker.containsKey(proposedTargetId)) {
      // Mettre à jour le compteur de votes pour cette cible
      targetVotes.put(proposedTargetId, targetVotes.getOrDefault(proposedTargetId, 0) + 1);
    }
    
    // Si c'est une nouvelle cible que nous ne connaissons pas encore
    else if (parts.length > 4) {
      // Essayer d'extraire les coordonnées si elles sont disponibles dans le message
      try {
        double enemyX = Double.parseDouble(parts[4]);
        double enemyY = Double.parseDouble(parts[5]);
        
        // Créer une entrée temporaire dans notre tracker
        enemyTracker.put(proposedTargetId, new EnemyInfo(enemyX, enemyY, 1000, null));
        
        // Ajouter un vote pour cette cible
        targetVotes.put(proposedTargetId, targetVotes.getOrDefault(proposedTargetId, 0) + 1);
      } catch (Exception e) {
        // Ignorer si les coordonnées ne sont pas valides
      }
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
    double targetAngle = Math.atan2(targetY - myY, targetX - myX);
    
    for (IRadarResult o : detectRadar()) {
        if ((o.getObjectType() == IRadarResult.Types.TeamMainBot ||
            o.getObjectType() == IRadarResult.Types.TeamSecondaryBot) &&
            o.getObjectDistance() <= 150) {
            
            // Vérifier si le coéquipier est dans la direction de notre mouvement
            double angleDiff = Math.abs(normalizeAngleDifference(o.getObjectDirection() - targetAngle));
            if (angleDiff < Math.PI/4) { // Tolérance de 45 degrés
                return true;
            }
        }
    }
    return false;
}
  
  // --- MÉTHODES D'ODOMÉTRIE ET DE DÉTECTION DE COLLISION ---
  private void updateOdometry() {
    boolean collision = detectCollision(COLLISION_THRESHOLD);
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
      estimatedX = kalmanFilter(predictedX, newX, uncertaintyX, true);
      estimatedY = kalmanFilter(predictedY, newY, uncertaintyY, false);
      
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
   * @param isXAxis Indique si l'axe est X (true) ou Y (false)
   * @return Nouvelle estimation filtrée
   */
  private double kalmanFilter(double predicted, double measured, double uncertainty, boolean isXAxis) {
    // Calculer le gain de Kalman standard
    double kalmanGain = uncertainty / (uncertainty + SENSOR_NOISE);
    
    // Ajuster le gain avec le poids spécifié pour la mesure
    double measurementWeight = 0.7; // Influence de la mesure directe
    kalmanGain = (1 - measurementWeight) * kalmanGain + measurementWeight;
    
    // Mise à jour de l'estimation
    double newEstimated = predicted + kalmanGain * (measured - predicted);
    
    // Mise à jour de l'incertitude
    double newUncertainty = (1 - kalmanGain) * uncertainty + PROCESS_NOISE;
    
    // Mettre à jour la variable d'incertitude correspondante
    if (isXAxis) {
        uncertaintyX = newUncertainty;
    } else {
        uncertaintyY = newUncertainty;
    }
    
    return newEstimated;
}
  
  private void refinePositionWithRadar() {
    for (IRadarResult o : detectRadar()) {
        if (o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 400) {
            // Obtenir la position du wreck dans le référentiel global
            double wreckX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
            double wreckY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
            
            // Utiliser cette information comme point d'ancrage pour ajuster notre position
            // au lieu de recalculer une position redondante
            estimatedX = kalmanFilter(estimatedX, myX, uncertaintyX * 1.5, true);
            estimatedY = kalmanFilter(estimatedY, myY, uncertaintyY * 1.5, false);
            
            myX = estimatedX;
            myY = estimatedY;
            break;
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
    
    String stateStr;
    switch (state) {
      case TRIANGLE_FORMATION: stateStr = "FORMATION"; break;
      case FINAL_ORIENTATION: stateStr = "ORIENT"; break;
      case MOVETASK: stateStr = "MOVE"; break;
      case DODGE: stateStr = "DODGE"; break;
      case HUNT_MODE: stateStr = "HUNT"; break;
      default: stateStr = ""+state;
    }
    
    sendLogMessage(prefix + " State=" + stateStr + 
                  " current=(" + (int) myX + ", " + (int) myY + 
                  "), teamDead=" + teamMemberDied +
                  ", target=" + (targetEnemyId != -1 ? "YES" : "NO"));
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
          return true;
        }
      }
    }
    return false;
  }
  
  private boolean detectCollision(double distance) {    
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectDistance() < distance && 
          o.getObjectType() != IRadarResult.Types.BULLET) {
        return true;
      }
    }
    return false;
  }

  // Remplacer la génération d'ID dans handleRadarDetection:
  private int getOrCreateEnemyId(double enemyX, double enemyY) {
    // Chercher un ennemi existant à proximité
    for (Map.Entry<Integer, double[]> entry : enemyPositions.entrySet()) {
        double[] pos = entry.getValue();
        double distance = Math.hypot(pos[0] - enemyX, pos[1] - enemyY);
        if (distance < 50) { // 50 est un seuil de distance raisonnable
            // Mettre à jour la position
            pos[0] = enemyX;
            pos[1] = enemyY;
            return entry.getKey();
        }
    }
    
    // Nouvel ennemi détecté
    int newId = nextEnemyId++;
    enemyPositions.put(newId, new double[]{enemyX, enemyY});
    return newId;
  }

  private void selectTarget() {
    long currentTime = System.currentTimeMillis();
    
    // Force target change if we've been focusing the same target too long
    boolean forceChange = (targetEnemyId != -1) && 
                         (currentTime - currentTargetStartTime > TARGET_MAX_FOCUS_TIME);
    
    // Si forceChange est true ou si la cible persistante n'est plus valide, on choisit une nouvelle cible
    if (forceChange || 
        persistentTargetId == -1 || 
        !enemyTracker.containsKey(persistentTargetId) ||
        currentTime - enemyTracker.get(persistentTargetId).lastSeen > TARGET_PERSISTENCE_TIME) {
        
        int bestTargetId = -1;
        double bestScore = 0;
        
        for (Map.Entry<Integer, EnemyInfo> entry : enemyTracker.entrySet()) {
            int enemyId = entry.getKey();
            EnemyInfo enemy = entry.getValue();
            
            // Skip current target if forcing change
            if (forceChange && enemyId == targetEnemyId) {
                continue;
            }
            
            double distance = enemy.distance;
            long timeSinceLastSeen = currentTime - enemy.lastSeen;
            
            // Score calculation - prioritize:
            // 1. Recently seen enemies (fresher data)
            // 2. MainBots over SecondaryBots
            // 3. Closer enemies over distant ones
            double freshnessScore = Math.max(0, 1.0 - (timeSinceLastSeen / (double)TARGET_PERSISTENCE_TIME));
            double typeScore = (enemy.type == IRadarResult.Types.OpponentMainBot) ? 2.0 : 1.0;
            double distanceScore = 1000.0 / Math.max(distance, 100.0); // Avoid division by very small numbers
            
            double score = freshnessScore * typeScore * distanceScore;
            
            if (score > bestScore) {
                bestScore = score;
                bestTargetId = enemyId;
            }
        }
        
        if (bestTargetId != -1 && (bestTargetId != targetEnemyId || forceChange)) {
            persistentTargetId = bestTargetId;
            targetEnemyId = bestTargetId;
            currentTargetStartTime = currentTime; // Reset the timer when changing targets
            
            EnemyInfo target = enemyTracker.get(targetEnemyId);
            sendLogMessage("Target changed to: " + target.type + 
                          " at (" + (int)target.x + "," + (int)target.y + 
                          "), reason: " + (forceChange ? "timeout" : "better target"));
        }
    }
}

private double[] calculateInterceptionPoint(EnemyInfo target) {
    // Calculer la vitesse et direction de la cible
    double targetVelX = target.x - target.lastX;
    double targetVelY = target.y - target.lastY;
    
    // Déterminer si la cible se déplace significativement
    boolean targetMoving = Math.hypot(targetVelX, targetVelY) > 1.0;
    
    if (!targetMoving) {
        // Si la cible est immobile, viser directement dessus
        return new double[] { target.x, target.y };
    }
    
    // Temps depuis la dernière mise à jour
    long timeDelta = System.currentTimeMillis() - target.lastSeen;
    
    // Notre vitesse maximale (estimation)
    double mySpeed = Parameters.teamAMainBotSpeed;
    
    // Vitesse de l'ennemi
    double enemySpeed = Math.hypot(targetVelX, targetVelY);
    
    // Position prédite avec le temps (prédiction linéaire simple)
    double predictedX = target.x + targetVelX * 5; // 5 étapes d'anticipation
    double predictedY = target.y + targetVelY * 5;
    
    // Si nous sommes plus rapides, calculer un point d'interception
    if (mySpeed > enemySpeed) {
        // Calculer un point entre notre position et la position prédite
        // qui favorise l'interception par devant
        double factor = 0.7; // 0.5 = milieu, >0.5 = plus proche de la cible
        double interceptX = myX + (predictedX - myX) * factor;
        double interceptY = myY + (predictedY - myY) * factor;
        
        return new double[] { interceptX, interceptY };
    } else {
        // Si l'ennemi est plus rapide, viser là où il sera
        return new double[] { predictedX, predictedY };
    }
}

private void moveToCoordinatesAggressive(double targetX, double targetY) {
    // Version plus directe de moveToCoordinates avec moins d'évitement
    
    double dx = targetX - myX;
    double dy = targetY - myY;
    double distance = Math.hypot(dx, dy);
    
    // Direction vers la cible
    double angleToTarget = normalizeHeading(Math.atan2(dy, dx));
    
    // Vérifier les obstacles critiques uniquement (très proches)
    for (IRadarResult o : detectRadar()) {
        if (o.getObjectType() != IRadarResult.Types.BULLET && 
            o.getObjectDistance() < COLLISION_THRESHOLD * 0.6) {
            
            // Seulement éviter les obstacles directement sur le chemin
            double angleDiff = Math.abs(normalizeAngleDifference(o.getObjectDirection() - angleToTarget));
            
            if (angleDiff < Math.PI/6) { // 30 degrés de tolérance seulement
                // Obstacle critique directement devant - appliquer évitement minimal
                double perpendicular = normalizeHeading(o.getObjectDirection() + Math.PI/2);
                double alt1 = perpendicular;
                double alt2 = normalizeHeading(perpendicular + Math.PI);
                
                // Choisir la direction qui nous rapproche le plus de la cible
                double diff1 = Math.abs(normalizeAngleDifference(alt1 - angleToTarget));
                double diff2 = Math.abs(normalizeAngleDifference(alt2 - angleToTarget));
                
                angleToTarget = (diff1 < diff2) ? alt1 : alt2;
                break;
            }
        }
    }
    
    // Se tourner et avancer dans la direction calculée
    turnAndMove(angleToTarget);
}

private double[] calculateFlankingPosition(double targetX, double targetY) {
    // Angle de base vers la cible
    double baseAngle = Math.atan2(targetY - myY, targetX - myX);
    
    // Décalage en fonction de l'identité du robot
    double offsetAngle = 0;
    if (whoAmI == ALPHA) offsetAngle = Math.PI / 4; // +45 degrés
    else if (whoAmI == GAMMA) offsetAngle = -Math.PI / 4; // -45 degrés
    // BETA attaque frontalement (offset = 0)
    
    // Calculer la position décalée à une certaine distance
    double flankDistance = 200; // Distance de flanquement
    double flankX = targetX - flankDistance * Math.cos(baseAngle + offsetAngle);
    double flankY = targetY - flankDistance * Math.sin(baseAngle + offsetAngle);
    
    return new double[] { flankX, flankY };
}

/**
 * Vérifie s'il y a un obstacle entre le robot et la cible
 * @param targetX Coordonnée X de la cible
 * @param targetY Coordonnée Y de la cible
 * @return true si la voie est libre, false si un obstacle est détecté
 */
private boolean isLineOfFireClear(double targetX, double targetY) {
    // Calculer l'angle et la distance à la cible
    double angle = Math.atan2(targetY - myY, targetX - myX);
    double distance = Math.hypot(targetX - myX, targetY - myY);
    
    // Vérifier tous les objets détectés par le radar
    for (IRadarResult o : detectRadar()) {
        // Ignorer les balles mais PAS les alliés
        if (o.getObjectType() == IRadarResult.Types.BULLET) {
            continue;
        }
        
        // Si c'est un allié, utiliser une tolérance d'angle plus large pour éviter tout risque
        boolean isTeammate = (o.getObjectType() == IRadarResult.Types.TeamMainBot || 
                             o.getObjectType() == IRadarResult.Types.TeamSecondaryBot);
        
        // Plus grande tolérance d'angle pour les coéquipiers (évite les tirs amis)
        double angleTolerance;
        if (isTeammate) {
            angleTolerance = 0.5; // ~30 degrés de marge pour les coéquipiers
        } else if (isEnemyBot(o.getObjectType())) {
            continue; // On veut tirer sur les ennemis, donc ils ne bloquent pas la ligne de tir
        } else {
            // Pour les autres obstacles (wreck, etc.)
            angleTolerance = 0.2 * (1.0 - o.getObjectDistance() / distance);
            angleTolerance = Math.max(0.05, Math.min(0.3, angleTolerance));
        }
        
        // Calculer l'écart d'angle entre l'objet et la direction de tir
        double angleDiff = Math.abs(normalizeAngleDifference(o.getObjectDirection() - angle));
        
        // Est-ce que l'objet est entre nous et la cible?
        if (angleDiff < angleTolerance && o.getObjectDistance() < distance) {
            if (isTeammate) {
                sendLogMessage("Tir annulé - coéquipier dans la ligne de tir");
            }
            return false;
        }
    }
    
    return true;
}

/**
 * Vérifie si un ennemi est à portée de tir immédiat et tire dessus
 * @return true si un tir a été effectué, false sinon
 */
private boolean fireAtNearbyEnemy() {
  // Si le cooldown de tir n'est pas terminé, ne pas tirer
  if (fireRythm > 0) return false;
  
  IRadarResult closestEnemy = null;
  double minDistance = IMMEDIATE_FIRING_THRESHOLD;
  
  // Rechercher l'ennemi le plus proche à portée de tir
  for (IRadarResult o : detectRadar()) {
    if (isEnemyBot(o.getObjectType()) && o.getObjectDistance() < minDistance) {
      // Ne tirer que si l'ennemi est devant nous (dans un angle de ±60°)
      double angleDiff = Math.abs(normalizeAngleDifference(o.getObjectDirection() - getHeading()));
      if (angleDiff < Math.PI/3) { // 60 degrés dans chaque direction
        minDistance = o.getObjectDistance();
        closestEnemy = o;
      }
    }
  }
  
  // Si un ennemi proche a été trouvé
  if (closestEnemy != null) {
    // Vérifier si la ligne de tir est dégagée
    double enemyX = myX + closestEnemy.getObjectDistance() * Math.cos(closestEnemy.getObjectDirection());
    double enemyY = myY + closestEnemy.getObjectDistance() * Math.sin(closestEnemy.getObjectDirection());
    
    if (isLineOfFireClear(enemyX, enemyY)) {
      // Tirer directement sur l'ennemi
      fire(closestEnemy.getObjectDirection());
      fireRythm = FIRE_COOLDOWN; // Réinitialiser le cooldown
      
      // Enregistrer cet ennemi dans le tracker
      int enemyId = getOrCreateEnemyId(enemyX, enemyY);
      if (enemyTracker.containsKey(enemyId)) {
        enemyTracker.get(enemyId).update(enemyX, enemyY, closestEnemy.getObjectDistance());
        enemyTracker.get(enemyId).type = closestEnemy.getObjectType();
      } else {
        enemyTracker.put(enemyId, new EnemyInfo(enemyX, enemyY, closestEnemy.getObjectDistance(), closestEnemy.getObjectType()));
      }
      
      // Partager cette information avec l'équipe
      sendMessage(FIRE, enemyX, enemyY, closestEnemy.getObjectDistance(), 
                closestEnemy.getObjectType() == IRadarResult.Types.OpponentMainBot ? 1 : 2);
      
      sendLogMessage("Tir immédiat sur ennemi à " + (int)closestEnemy.getObjectDistance() + " unités");
      return true;
    }
  }
  
  return false;
}

/**
 * Partage périodiquement la cible préférée de ce robot avec le reste de l'équipe
 */
private void sharePreferedTarget() {
  long currentTime = System.currentTimeMillis();
  
  // Vérifier si c'est le moment de diffuser notre cible
  if (currentTime - lastTargetBroadcastTime > TARGET_BROADCAST_INTERVAL && targetEnemyId != -1) {
    // Envoyer notre préférence de cible à l'équipe
    sendMessage(TARGET_CONSENSUS, targetEnemyId);
    lastTargetBroadcastTime = currentTime;
    
    // Debug
    if (teamConsensusTargetId != -1) {
      sendLogMessage("Ma cible: " + targetEnemyId + ", Consensus équipe: " + teamConsensusTargetId);
    }
  }
}

/**
 * Met à jour la cible consensuelle de l'équipe en fonction des votes
 */
private void updateTeamConsensusTarget() {
  // Si nous n'avons pas de votes, garder la cible actuelle
  if (targetVotes.isEmpty()) {
    return;
  }
  
  // Trouver la cible avec le plus de votes
  int maxVotes = 0;
  int mostVotedTarget = -1;
  
  for (Map.Entry<Integer, Integer> entry : targetVotes.entrySet()) {
    if (entry.getValue() > maxVotes) {
      maxVotes = entry.getValue();
      mostVotedTarget = entry.getKey();
    }
  }
  
  // Si nous avons une cible avec suffisamment de votes (au moins 2)
  if (maxVotes >= 2) {
    teamConsensusTargetId = mostVotedTarget;
    
    // Si notre cible actuelle n'est pas celle du consensus et que le consensus est valide
    if (targetEnemyId != teamConsensusTargetId && enemyTracker.containsKey(teamConsensusTargetId)) {
      // Adopter la cible consensuelle
      targetEnemyId = teamConsensusTargetId;
      
      EnemyInfo target = enemyTracker.get(targetEnemyId);
      targetX = target.x;
      targetY = target.y;
      
      sendLogMessage("Adoption de la cible consensuelle: " + targetEnemyId);
    }
  }
  
  // Nettoyer les votes obsolètes (plus anciens que 3 secondes)
  long currentTime = System.currentTimeMillis();
  targetVotes.entrySet().removeIf(entry -> 
      !enemyTracker.containsKey(entry.getKey()) || 
      currentTime - enemyTracker.get(entry.getKey()).lastSeen > 3000);
}
}
