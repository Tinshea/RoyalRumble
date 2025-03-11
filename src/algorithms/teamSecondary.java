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

  private static final int ROCKY = 0x1EADDA;
  private static final int MARIO = 0x5EC0;
  private static final int TEAM = 0xBADDAD;
  private static final int UNDEFINED = 0xBADC0DE0;

  private static final int FIRE = 0xB52;
  private static final int FALLBACK = 0xFA11BAC;
  private static final int ROGER = 0x0C0C0C0C;
  private static final int OVER = 0xC00010FF;

  // ---Tâches---//
  // ---Position initiale de l'équipe---//
  private static final int TURNLEFTTASKINIT1 = 1;
  private static final int TURNRIGHTTASKINIT1 = 2;
  private static final int TURNLEFTTASKINIT2 = 3;
  private static final int TURNRIGHTTASKINIT2 = 4;

  private static final int MOVETASK = 5;
  private static final int SINK = 0xBADC0DE1;

  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  // ---VARIABLES---//
  private int state;
  private double oldAngle;
  private double myX, myY;
  private boolean isMoving;
  private int whoAmI;
  private int myTeam; // Pour stocker l'équipe du robot
  private ArrayList<String> receivedMessages; // Pour stocker les messages reçus

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
    oldAngle = getHeading();
    receivedMessages = new ArrayList<>(); // Initialisation de la liste de messages
  }

  public void step() {
    // ODOMETRY CODE
    if (isMoving) {
      myX += Parameters.teamASecondaryBotSpeed * Math.cos(getHeading());
      myY += Parameters.teamASecondaryBotSpeed * Math.sin(getHeading());
      isMoving = false;
    }
    // DEBUG MESSAGE
    if (whoAmI == ROCKY) {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#ROCKY " + teamName + " State: " + state + " south :" + getHeading());
    } else {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#MARIO " + teamName + " State: " + state + " north :" + getHeading());
    }

    // RADAR DETECTION
    // s'arrete si il est trop proche d'un ennemi
    for (IRadarResult o : detectRadar()) {
      if (o.getObjectType() == IRadarResult.Types.OpponentMainBot
          || o.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
        double enemyX = myX + o.getObjectDistance() * Math.cos(o.getObjectDirection());
        double enemyY = myY + o.getObjectDistance() * Math.sin(o.getObjectDirection());
        sendMessage(FIRE, enemyX, enemyY);
      }
      if (o.getObjectDistance() <= 100) {

      }
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
      return;
    }
    if (true) {
      return;
    }
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

  private void myMove() {
    isMoving = true;
    move();
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

    for (Object datum : data) {
      message.append(":").append(datum);
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
        // Traite un message de tir
        // Le robot secondaire ne tire pas mais peut réagir d'une autre façon
        sendLogMessage("Received FIRE message at coordinates: " + parts[3] + ", " + parts[4]);
        break;

      case FALLBACK:
        // Traite un message de repli
        // À implémenter selon les besoins
        break;

      case ROGER:
        // Traite un message d'acquittement
        // À implémenter selon les besoins
        break;

      default:
        // Message de type inconnu
        sendLogMessage("Unknown message type: " + messageType);
        break;
    }
  }
}