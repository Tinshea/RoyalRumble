/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
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

public class teamSecondary extends Brain {
  // ---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.01;
  private static final double COLLISION_THRESHOLD = 110;
  private static final double ARENA_WIDTH = 3000;
  private static final double ARENA_HEIGHT = 2000;
  private static final double SAFETY_MARGIN = 100.0;

  private static final int ROCKY = 0x1EADDA;
  private static final int MARIO = 0x5EC0;
  private static final int TEAM = 0xBADDAD;
  private static final int UNDEFINED = 0xBADC0DE0;

  private static final int FIRE = 0xB52;
  private static final int FALLBACK = 0xFA11BAC;
  private static final int ROGER = 0x0C0C0C0C;
  private static final int COMBAT = 0xB52B52;
  private static final int OVER = 0xC00010FF;
  
  // ---Tâches---//
  // ---Position initiale de l'équipe---//
  private static final int TURNLEFTTASKINIT1 = 1;
  private static final int TURNRIGHTTASKINIT1 = 2;
  private static final int TURNLEFTTASKINIT2 = 3;
  private static final int TURNRIGHTTASKINIT2 = 4;

  private static final int MOVETASK = 5;
  private static final int FOLLOWTASK = 6;
  private static final int FLEE = 7;
  private static final int SINK =  0xBADC0DE1;

  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  private static final double SENSOR_NOISE = 2.0;
  private static final double PROCESS_NOISE = 3.0;

  // ---VARIABLES---//
  private int state;
  private double oldAngle;
  private double myX, myY;
  private boolean isMoving;
  private boolean isMovingBack;
  private int whoAmI;
  private int myTeam; // Pour stocker l'équipe du robot
  private ArrayList<String> receivedMessages; // Pour stocker les messages reçus

  private double estimatedX, estimatedY;
  private double uncertaintyX = 1.0, uncertaintyY = 1.0;
  private double velocityX = 0.0, velocityY = 0.0;
  private double lastUpdateTime = 0;

  // ---CONSTRUCTORS---//
  public teamSecondary() {
    super();
  }

  // ---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    // ODOMETRY CODE
    whoAmI = ROCKY;
    for (IRadarResult o : detectRadar())
      if (isSameDirection(o.getObjectDirection(), Parameters.NORTH))
        whoAmI = UNDEFINED;
    if (whoAmI == ROCKY) {
      myX = Parameters.teamASecondaryBot1InitX;
      myY = Parameters.teamASecondaryBot1InitY;
    } else {
      myX = Parameters.teamASecondaryBot2InitX;
      myY = Parameters.teamASecondaryBot2InitY;
    }

    // Déterminer l'équipe
    myTeam = determineTeam();

    // INIT
    if (myTeam == TEAM_A) {
      state = (whoAmI == ROCKY) ? TURNLEFTTASKINIT1 : TURNRIGHTTASKINIT1;
    } else {
      state = (whoAmI == ROCKY) ? TURNRIGHTTASKINIT1 : TURNLEFTTASKINIT1;
    }
    isMoving = false;
    isMovingBack = false;
    oldAngle = getHeading();
    receivedMessages = new ArrayList<>(); // Initialisation de la liste de messages

    estimatedX = myX;
    estimatedY = myY;
    velocityX = 0.0;
    velocityY = 0.0;
    lastUpdateTime = System.currentTimeMillis();
  }

  public void step() {
    // Update odometry first
    updateOdometry();
    
    // DEBUG MESSAGE
    if (whoAmI == ROCKY) {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#ROCKY " + teamName + " State: " + state + " Position: (" + (int)myX + "," + (int)myY + ")");
    } else {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#MARIO " + teamName + " State: " + state + " Position: (" + (int)myX + "," + (int)myY + ")");
    }
    
    // RADAR DETECTION
    // Track if we found enemy in this radar scan
    boolean enemyDetected = false;

    for (IRadarResult o : detectRadar()) {
      // Detect and dodge bullets - highest priority
      if (o.getObjectType() == IRadarResult.Types.BULLET) {
        // Log the bullet distance and take evasive action
        double bulletDistance = o.getObjectDistance();
        // System.out.println("BULLET DETECTED! Distance: " + bulletDistance);
        
        // Dodge bullet by moving perpendicular to its direction
        stepTurn(Parameters.Direction.RIGHT);
      }

      // Detect enemies and send fire message
      if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
        || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
        double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        sendMessage(FIRE, enemyX, enemyY);
        enemyDetected = true;
      }
      
      // Check for wrecks - if an enemy became a wreck, it's dead
      if (o.getObjectType() == IRadarResult.Types.Wreck && state == FOLLOWTASK) {
        sendLogMessage("Enemy destroyed! Looking for new targets");
        state = MOVETASK; // Reset to movement state to find new enemies
      }
      
      // Follow MainBot if detected
      if (o.getObjectType() == IRadarResult.Types.OpponentMainBot) {
        state = FOLLOWTASK;
      }
      
      // Handle orientation and movement towards enemy
      if (state == FOLLOWTASK && o.getObjectType() == IRadarResult.Types.OpponentMainBot) {
        // Track enemy as it moves
        double enemyDirection = o.getObjectDirection();
        double headingDiff = normalizeAngle(enemyDirection - getHeading());
        
        // Always adjust orientation to keep tracking the enemy
        // Even for small deviations in enemy movement
        if (Math.abs(headingDiff) > 0.2) {
          // Turn towards enemy - continuous tracking
          if (headingDiff > 0) {
            stepTurn(Parameters.Direction.RIGHT);
          } else {
            stepTurn(Parameters.Direction.LEFT);
          }
          sendLogMessage("Tracking enemy movement");
          return; // Exit to complete tracking adjustment first
        }
        
        // Then decide whether to approach or back away
        if (o.getObjectDistance() <= 500) {
          myMoveBack();
        } else {
          myMove();
        }
      }
    }
    
    // If we're following an enemy but didn't detect it this scan, reset to search mode
    if (state == FOLLOWTASK && !enemyDetected) {
      sendLogMessage("Enemy lost! Searching for new target");
      state = MOVETASK;
    }

    // COMMUNICATION
    ArrayList<String> messages = fetchAllMessages();
    receivedMessages.clear(); // Efface les anciens messages à chaque step
    for (String m : messages) {
      if (Integer.parseInt(m.split(":")[1]) == whoAmI || Integer.parseInt(m.split(":")[1]) == TEAM) {
        receivedMessages.add(m); // Stocke les messages pertinents
        process(m);
      }
    }

    // --- AUTOMATE DE POSITIONNEMENT ---//
    if (state == TURNLEFTTASKINIT1) {
      double targetDirection = (myTeam == TEAM_A) ? Parameters.NORTH : Parameters.SOUTH;
      if (!isSameDirection(getHeading(), targetDirection)) {
        stepTurn(Parameters.Direction.LEFT);
        return;
      }
      state = MOVETASK;
      myMove();
      return;
    }

    if (state == TURNRIGHTTASKINIT1) {
      double targetDirection = (myTeam == TEAM_A) ? Parameters.SOUTH : Parameters.NORTH;
      if (!isSameDirection(getHeading(), targetDirection)) {
        stepTurn(Parameters.Direction.RIGHT);
        return;
      }
      state = MOVETASK;
      myMove();
      return;
    }

    if (state == MOVETASK) {
      if (detectFront().getObjectType() == IFrontSensorResult.Types.NOTHING) {
        myMove();
      } else {
        if (myTeam == TEAM_A) {
          state = (whoAmI == ROCKY) ? TURNRIGHTTASKINIT2 : TURNLEFTTASKINIT2;
          stepTurn((whoAmI == ROCKY) ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
        } else {
          state = (whoAmI == ROCKY) ? TURNLEFTTASKINIT2 : TURNRIGHTTASKINIT2;
          stepTurn((whoAmI == ROCKY) ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT);
        }
        oldAngle = getHeading();
      }
      return;
    }

    if ((state == TURNRIGHTTASKINIT2 || state == TURNLEFTTASKINIT2) && !isSameDirection(getHeading(),
        oldAngle + (state == TURNRIGHTTASKINIT2 ? Parameters.RIGHTTURNFULLANGLE : -Parameters.RIGHTTURNFULLANGLE))) {
      stepTurn(state == TURNRIGHTTASKINIT2 ? Parameters.Direction.RIGHT : Parameters.Direction.LEFT);
      return;
    }
    if ((state == TURNRIGHTTASKINIT2 || state == TURNLEFTTASKINIT2) && isSameDirection(getHeading(),
        oldAngle + (state == TURNRIGHTTASKINIT2 ? Parameters.RIGHTTURNFULLANGLE : -Parameters.RIGHTTURNFULLANGLE))) {
      state = SINK;
      myMove();
      return;
    }

    // --- AUTOMATE FIN DE POSITIONNEMENT ---//

    if (state == SINK) {
      state = MOVETASK;
      return;
    }
    if (true) {
      return;
    }
  }

  // --- MÉTHODES D'ODOMÉTRIE ET DE DÉTECTION DE COLLISION ---
  private void updateOdometry() {
    boolean collision = detectCollision();
    if (isMoving) {
      updatePosition(Parameters.teamASecondaryBotSpeed, collision);
      isMoving = false;
    } else if (isMovingBack) {
      updatePosition(-Parameters.teamASecondaryBotSpeed, collision);
      isMovingBack = false;
    }
    
    refinePositionWithRadar();
  }
  
  private void updatePosition(double speed, boolean collision) {
    if (!collision) {
      long currentTime = System.currentTimeMillis();
      double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
      if (deltaTime <= 0) deltaTime = 0.01;
      lastUpdateTime = currentTime;
      
      double newX = myX + speed * Math.cos(getHeading());
      double newY = myY + speed * Math.sin(getHeading());
      
      double instantVelocityX = (newX - myX) / deltaTime;
      double instantVelocityY = (newY - myY) / deltaTime;
      
      double alpha = 0.3;
      velocityX = alpha * instantVelocityX + (1 - alpha) * velocityX;
      velocityY = alpha * instantVelocityY + (1 - alpha) * velocityY;
      
      double predictedX = estimatedX + velocityX * deltaTime;
      double predictedY = estimatedY + velocityY * deltaTime;
      
      double measurementWeight = 0.7;
      estimatedX = kalmanFilter(predictedX, newX, uncertaintyX, measurementWeight);
      estimatedY = kalmanFilter(predictedY, newY, uncertaintyY, measurementWeight);
      
      myX = estimatedX;
      myY = estimatedY;
      
      if (speed != 0) {
        sendLogMessage("Vitesse: vX=" + Math.round(velocityX) + ", vY=" + Math.round(velocityY));
      }
    } else if (speed < 0) {
      sendLogMessage("Collision détectée - position non mise à jour");
      velocityX = 0;
      velocityY = 0;
    }
  }
  
  private double kalmanFilter(double predicted, double measured, double uncertainty, double measurementWeight) {
    double kalmanGain = uncertainty / (uncertainty + SENSOR_NOISE);
    kalmanGain = (1 - measurementWeight) * kalmanGain + measurementWeight;
    double newEstimated = predicted + kalmanGain * (measured - predicted);
    double newUncertainty = (1 - kalmanGain) * uncertainty + PROCESS_NOISE;
    
    if (predicted == estimatedX) {
      uncertaintyX = newUncertainty;
    } else {
      uncertaintyY = newUncertainty;
    }
    
    return newEstimated;
  }
  
  private void refinePositionWithRadar() {
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == IRadarResult.Types.Wreck && o.getObjectDistance() < 400) {
        double wreckX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double wreckY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        double robotX = wreckX - o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double robotY = wreckY - o.getObjectDistance() * Math.sin(o.getObjectDirection());
        
        estimatedX = kalmanFilter(estimatedX, robotX, uncertaintyX * 1.5, 0.3);
        estimatedY = kalmanFilter(estimatedY, robotY, uncertaintyY * 1.5, 0.3);
        
        myX = estimatedX;
        myY = estimatedY;
        break;
      }
    }
  }

  private boolean detectCollision() {
    for (IRadarResult o : detectRadar()) {
      if (isMoving && o.getObjectDistance() < COLLISION_THRESHOLD &&
          isRoughlySameDirection(o.getObjectDirection(), getHeading())) {
        return true;
      }
    }
    return false;
  }

  // ---UTILITIES---//
  // Normaliser un angle pour qu'il soit entre -PI et PI
  private double normalizeAngle(double angle) {
    while (angle > Math.PI)
      angle -= 2 * Math.PI;
    while (angle < -Math.PI)
      angle += 2 * Math.PI;
    return Math.abs(angle);
  }

  private boolean isSameDirection(double dir1, double dir2) {
    double diff = normalizeAngle(dir1 - dir2);
    return diff < ANGLEPRECISION;
  }
  
  private boolean isRoughlySameDirection(double dir1, double dir2) {
    return Math.abs(normalizeAngle(dir1) - normalizeAngle(dir2)) < Math.PI / 6.0; // Using pi/6 (30 degrees) for rough comparison
  }

  private void myMove() {
    isMoving = true;
    move();
  }
  
  private void myMoveBack() {
    isMovingBack = true;
    moveBack();
  }

  /**
   * Détermine à quelle équipe appartient ce robot
   * 
   * @return TEAM_A ou TEAM_B
   */
  private int determineTeam() {
    if (isSameDirection(getHeading(), Parameters.EAST)) {
      return TEAM_A;
    }
    return TEAM_B;
  }

  /**
   * Sends a formatted message through broadcast
   * 
   * @param messageType Type of message (FIRE, FALLBACK, etc.)
   * @param data        Array of data to include in the message
   */
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

  /**
   * Traite un message reçu en fonction de son type
   * 
   * @param message Le message à traiter
   */
  private void process(String message) {
    String[] parts = message.split(":");
    int messageType = Integer.parseInt(parts[2]);
    switch (messageType) {
      case FIRE:
        // Do nothing 
        break;

      case FALLBACK:
        // Traite un message de repli
        // À implémenter selon les besoins
        break;

      case ROGER:
        // Traite un message d'acquittement
        // À implémenter selon les besoins
        break;
      
      case COMBAT: 
        state = MOVETASK;
        break;

      default:
        // Message de type inconnu
        sendLogMessage("Unknown message type: " + messageType);
        break;
    }
  }
}