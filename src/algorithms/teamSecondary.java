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
  //---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.01;

  private static final int ROCKY = 0x1EADDA;
  private static final int MARIO = 0x5EC0;
  private static final int TEAM = 0xBADDAD;
  private static final int UNDEFINED = 0xBADC0DE0;
  
  private static final int FIRE = 0xB52;
  private static final int FALLBACK = 0xFA11BAC;
  private static final int ROGER = 0x0C0C0C0C;
  private static final int OVER = 0xC00010FF;

  private static final int TURNLEFTTASK = 1;
  private static final int MOVETASK = 2;
  private static final int TURNRIGHTTASK = 3;
  private static final int SINK = 0xBADC0DE1;

  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  //---VARIABLES---//
  private int state;
  private double oldAngle;
  private double myX,myY;
  private boolean isMoving;
  private boolean freeze;
  private int arenaWidth = 3000;
  private int whoAmI;
  private int myTeam; // Pour stocker l'équipe du robot
  private ArrayList<String> receivedMessages; // Pour stocker les messages reçus

  //---CONSTRUCTORS---//
  public teamSecondary() { super(); }

  //---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    //ODOMETRY CODE
    whoAmI = ROCKY;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.NORTH)) whoAmI=UNDEFINED;
    if (whoAmI == ROCKY){
      myX=Parameters.teamASecondaryBot1InitX;
      myY=Parameters.teamASecondaryBot1InitY;
    } else {
      myX=Parameters.teamASecondaryBot2InitX;
      myY=Parameters.teamASecondaryBot2InitY;
    }

    // Déterminer l'équipe
    myTeam = determineTeam();

    //INIT
    state=TURNLEFTTASK;
    isMoving=false;
    oldAngle=getHeading();
    receivedMessages = new ArrayList<>(); // Initialisation de la liste de messages
  }

  /**
   * Détermine à quelle équipe appartient ce robot
   * @return TEAM_A ou TEAM_B
   */
   private int determineTeam() {

      if (isSameDirection(getHeading(),Parameters.EAST)) {
        return TEAM_A;
      } 
  
   return TEAM_B;
  }

  public void step() {
    //ODOMETRY CODE
    if (isMoving){
      myX+=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
      myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
      isMoving=false;
    }
    //DEBUG MESSAGE
    if (whoAmI == ROCKY) {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#ROCKY [" + teamName + "] *thinks* he is rolling at position (" + (int)myX + ", " + (int)myY + ").");
    } else {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#MARIO [" + teamName + "] *thinks* he is rolling at position (" + (int)myX + ", " + (int)myY + ").");
    }

    //RADAR DETECTION
    freeze=false;
    for (IRadarResult o: detectRadar()){
      if (o.getObjectType()==IRadarResult.Types.OpponentMainBot || o.getObjectType()==IRadarResult.Types.OpponentSecondaryBot) {
        double enemyX=myX+o.getObjectDistance()*Math.cos(o.getObjectDirection());
        double enemyY=myY+o.getObjectDistance()*Math.sin(o.getObjectDirection());
        sendMessage(FIRE, enemyX, enemyY);
      }
      if (o.getObjectDistance()<=100) {
        freeze=true;
      }
    }
    if (freeze) return;

    //COMMUNICATION
    ArrayList<String> messages = fetchAllMessages();
    receivedMessages.clear(); // Efface les anciens messages à chaque step
    for (String m: messages) {
      if (Integer.parseInt(m.split(":")[1]) == whoAmI || Integer.parseInt(m.split(":")[1]) == TEAM) {
        receivedMessages.add(m); // Stocke les messages pertinents
        process(m);
      }
    }

    //AUTOMATON
    if (state==TURNLEFTTASK && !(isSameDirection(getHeading(),Parameters.NORTH))) {
      stepTurn(Parameters.Direction.LEFT);
      //sendLogMessage("Initial TeamA Secondary Bot1 position. Heading North!");
      return;
    }
    if (state==TURNLEFTTASK && isSameDirection(getHeading(),Parameters.NORTH)) {
      state=MOVETASK;
      myMove();
      //sendLogMessage("Moving a head. Waza!");
      return;
    }
    if (state==MOVETASK && detectFront().getObjectType()==IFrontSensorResult.Types.NOTHING) {
      myMove(); //And what to do when blind blocked?
      //sendLogMessage("Moving a head. Waza!");
      return;
    }
    if (state==MOVETASK && detectFront().getObjectType()!=IFrontSensorResult.Types.NOTHING) {
      state=TURNRIGHTTASK;
      oldAngle=getHeading();
      stepTurn(Parameters.Direction.RIGHT);
      //sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
      return;
    }
    if (state==TURNRIGHTTASK && !(isSameDirection(getHeading(),oldAngle+Parameters.RIGHTTURNFULLANGLE))) {
      stepTurn(Parameters.Direction.RIGHT);
      //sendLogMessage("Iceberg at 12 o'clock. Heading 3!");
      return;
    }
    if (state==TURNRIGHTTASK && isSameDirection(getHeading(),oldAngle+Parameters.RIGHTTURNFULLANGLE)) {
      state=MOVETASK;
      myMove();
      //sendLogMessage("Moving a head. Waza!");
      return;
    }

    if (state==SINK) {
      myMove();
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
  private boolean isSameDirection(double dir1, double dir2){
    return Math.abs(dir1-dir2)<ANGLEPRECISION;
  }

  /**
   * Sends a formatted message through broadcast
   * @param messageType Type of message (FIRE, FALLBACK, etc.)
   * @param data Array of data to include in the message
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