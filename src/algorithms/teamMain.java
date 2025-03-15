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

public class teamMain extends Brain {
  //---PARAMETERS---//
  private static final double ANGLEPRECISION = 0.05;
  private static final double FIREANGLEPRECISION = Math.PI/(double)6;

  private static final int ALPHA = 0x1EADDA;
  private static final int BETA = 0x5EC0;
  private static final int GAMMA = 0x333;
  private static final int TEAM = 0xBADDAD;
  private static final int UNDEFINED = 0xBADC0DE0;
  
  private static final int FIRE = 0xB52;
  private static final int FALLBACK = 0xFA11BAC;
  private static final int ROGER = 0x0C0C0C0C;
  private static final int COMBAT = 0xB52B52;
  private static final int OVER = 0xC00010FF;
  private static final int DODGE = 0xD0D6E;

  private static final int TURNSOUTHTASK = 1;
  private static final int MOVETASK = 2;
  private static final int TURNLEFTTASK = 3;
  private static final int SINK = 0xBADC0DE1;
  
  // Nouveaux états pour la formation triangulaire
  private static final int TRIANGLE_FORMATION = 4;
  private static final int MOVE_TO_POSITION = 5;
  private static final int TURN_TO_POSITION = 6;
  private static final int FINAL_ORIENTATION = 7; // Nouvel état pour l'orientation finale
  
  private static final int TEAM_A = 0;
  private static final int TEAM_B = 1;

  // Paramètres de la formation triangulaire
  private static final double TRIANGLE_SIDE = 50; // Distance entre les robots

  //---VARIABLES---//
  private int state;
  private double oldAngle;
  private double myX,myY;
  private boolean isMoving;
  private int whoAmI;
  private int fireRythm,rythm,counter;
  private int countDown;
  private double targetX,targetY;
  private boolean fireOrder;
  private boolean freeze;
  private boolean friendlyFire;
  private ArrayList<String> receivedMessages; // Pour stocker les messages reçus
  private int myTeam; // Pour stocker l'équipe du robot
  
  // Variables pour la formation triangulaire
  private double formationX, formationY; // Position cible dans la formation
  private boolean positionReached = false;
  

  //---CONSTRUCTORS---//
  public teamMain() { super(); }

  //---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
    //ODOMETRY CODE
    whoAmI = GAMMA;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.NORTH)) whoAmI=ALPHA;
    for (IRadarResult o: detectRadar())
      if (isSameDirection(o.getObjectDirection(),Parameters.SOUTH) && whoAmI!=GAMMA) whoAmI=BETA;
    if (whoAmI == GAMMA){
      myX=Parameters.teamAMainBot1InitX;
      myY=Parameters.teamAMainBot1InitY;
    } else {
      myX=Parameters.teamAMainBot2InitX;
      myY=Parameters.teamAMainBot2InitY;
    }
    if (whoAmI == ALPHA){
      myX=Parameters.teamAMainBot3InitX;
      myY=Parameters.teamAMainBot3InitY;
    }

    // Déterminer l'équipe
    myTeam = determineTeam();

    //INIT
    state = TRIANGLE_FORMATION; // On commence par la formation en triangle
    isMoving=false;
    fireOrder=false;
    fireRythm=0;
    oldAngle=getHeading();
    receivedMessages = new ArrayList<>(); // Initialisation de la liste de messages
    
    // Définir les positions cibles pour la formation triangulaire
    double centerX = myX + (myTeam == TEAM_A ? 100 : -100);
    double centerY = myY;
    
    if (whoAmI == BETA) {
      formationX = centerX;
      formationY = centerY;
    } else if (whoAmI == GAMMA) {
      formationX = centerX - TRIANGLE_SIDE ;
      formationY = centerY - TRIANGLE_SIDE * Math.sqrt(3) / 2;
    } else if (whoAmI == ALPHA) {
      formationX = centerX - TRIANGLE_SIDE ;
      formationY = centerY + TRIANGLE_SIDE * Math.sqrt(3) / 2;
    }
    
    targetX = formationX;
    targetY = formationY;
  }

  public void step() {
    //ODOMETRY CODE
    if (isMoving){
      myX+=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
      myY+=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
      isMoving=false;
    }
    //DEBUG MESSAGE
    boolean debug=true;
    if (debug && whoAmI == ALPHA && state!=SINK) {
      String teamName = (myTeam == TEAM_A) ? "Team A" : "Team B";
      sendLogMessage("#ALPHA. #State= " + state + " target= (" + (int)targetX + ", " + (int)targetY + ") current= (" + (int)myX + ", " + (int)myY + ")");
    }
    if (debug && whoAmI == BETA && state!=SINK) {
      sendLogMessage("#BETA. #State= " + state + " target= (" + (int)targetX + ", " + (int)targetY + ") current= (" + (int)myX + ", " + (int)myY + ")");
    }
    if (debug && whoAmI == GAMMA && state!=SINK) {
      sendLogMessage("#GAMMA. #State= " + state + " target= (" + (int)targetX + ", " + (int)targetY + ") current= (" + (int)myX + ", " + (int)myY + ")");
    }
    if (debug && fireOrder) sendLogMessage("Firing enemy!!");

    //COMMUNICATION
    ArrayList<String> messages=fetchAllMessages();
    receivedMessages.clear(); // Efface les anciens messages à chaque step
    for (String m: messages) {
      if (Integer.parseInt(m.split(":")[1]) == whoAmI || Integer.parseInt(m.split(":")[1]) == TEAM) {
        receivedMessages.add(m); // Stocke les messages pertinents
        process(m);
      }
    }
    
    //RADAR DETECTION
    freeze=false;
    friendlyFire=true;
    for (IRadarResult o: detectRadar()){
      if (o.getObjectType()==IRadarResult.Types.OpponentMainBot || o.getObjectType()==IRadarResult.Types.OpponentSecondaryBot) {
        double enemyX=myX+o.getObjectDistance()*Math.cos(o.getObjectDirection());
        double enemyY=myY+o.getObjectDistance()*Math.sin(o.getObjectDirection());
        sendMessage(FIRE, enemyX, enemyY);
      }
      if (o.getObjectDistance()<=100 && !isRoughlySameDirection(o.getObjectDirection(),getHeading()) && o.getObjectType()!=IRadarResult.Types.BULLET) {
        freeze=true;
      }
      if (o.getObjectType()==IRadarResult.Types.TeamMainBot || o.getObjectType()==IRadarResult.Types.TeamSecondaryBot || o.getObjectType()==IRadarResult.Types.Wreck) {
        if (fireOrder && onTheWay(o.getObjectDirection())) {
          friendlyFire=false;
        }
      }
    }
    if (freeze) return;

    //AUTOMATON
    if (fireOrder) countDown++;
    if (countDown>=100) fireOrder=false;
    if (fireOrder && fireRythm==0 && friendlyFire) {
      firePosition(targetX,targetY);
      fireRythm++;
      return;
    }
    fireRythm++;
    if (fireRythm>=Parameters.bulletFiringLatency) fireRythm=0;
    
    // Formation triangulaire
    if (state == TRIANGLE_FORMATION) {
      // Vérifier si la position est atteinte
      double distToTarget = Math.sqrt(Math.pow(myX - formationX, 2) + Math.pow(myY - formationY, 2));
      
      if (distToTarget < 20) { // Si on est assez proche de la position cible
        positionReached = true;
        state = FINAL_ORIENTATION; // Passer à l'orientation finale au lieu de SINK
        return;
      } else {
        // Utiliser la nouvelle fonction pour se déplacer vers la cible
        moveToCoordinates(formationX, formationY);
        return;
      }
    }
    
    // Nouvel état pour l'orientation finale selon l'équipe
    if (state == FINAL_ORIENTATION) {
      double targetDirection = (myTeam == TEAM_A) ? Parameters.EAST : Parameters.WEST;
      
      if (isSameDirection(getHeading(), targetDirection)) {
        // Orientation finale atteinte
        state = MOVETASK;
        sendMessage(COMBAT, (Object) null);
        return;
      } else {
        // Déterminer le sens de rotation pour atteindre l'orientation finale
        double angleDiff = normalizeAngle(targetDirection - getHeading());
        if (angleDiff < Math.PI) {
          stepTurn(Parameters.Direction.RIGHT);
        } else {
          stepTurn(Parameters.Direction.LEFT);
        }
        return;
      }
    }

    if (state != DODGE && (detectFront().getObjectType() == IFrontSensorResult.Types.WALL || detectFront().getObjectType() == IFrontSensorResult.Types.Wreck)) {
      state = DODGE;
      IFrontSensorResult frontSensorResult = detectFront();
      double obstacleX = myX + Parameters.teamAMainBotSpeed * Math.cos(getHeading());
      double obstacleY = myY + Parameters.teamAMainBotSpeed * Math.sin(getHeading());
      sendMessage(DODGE, obstacleX, obstacleY);
      sendLogMessage("DODGE state activated. Obstacle detected at (" + obstacleX + ", " + obstacleY + ")");
      return;
    }

    if(state == DODGE){
      moveBack();
      return;
    }
    
    // États existants
    if (state==TURNSOUTHTASK && !(isSameDirection(getHeading(),Parameters.SOUTH))) {
      stepTurn(Parameters.Direction.RIGHT);
      return;
    }
    if (state==TURNSOUTHTASK && isSameDirection(getHeading(),Parameters.SOUTH)) {
      state=MOVETASK;
      myMove();
      return;
    }
    if (state==MOVETASK && detectFront().getObjectType()!=IFrontSensorResult.Types.WALL) {
      myMove();
      return;
    }
    if (state==MOVETASK && detectFront().getObjectType()==IFrontSensorResult.Types.WALL) {
      state=TURNLEFTTASK;
      oldAngle=getHeading();
      stepTurn(Parameters.Direction.LEFT);
      return;
    }
    if (state==TURNLEFTTASK && !(isSameDirection(getHeading(),oldAngle+Parameters.LEFTTURNFULLANGLE))) {
      stepTurn(Parameters.Direction.LEFT);
      return;
    }
    if (state==TURNLEFTTASK && isSameDirection(getHeading(),oldAngle+Parameters.LEFTTURNFULLANGLE)) {
      state=MOVETASK;
      myMove();
      return;
    }



    if (state==FIRE){
      if (fireRythm==0) {
        firePosition(700,1500);
        fireRythm++;
        return;
      }
      fireRythm++;
      if (fireRythm==Parameters.bulletFiringLatency) fireRythm=0;
      if (rythm==0) stepTurn(Parameters.Direction.LEFT); else myMove();
      rythm++;
      if (rythm==14) rythm=0;
      return;
    }

    if (state==SINK) {
      return;
    }
    if (true) {
      return;
    }
  }
  
  /**
   * Déplace le robot vers les coordonnées spécifiées
   * @param targetX Coordonnée X cible
   * @param targetY Coordonnée Y cible
   * @return true si un mouvement a été effectué, false sinon
   */
  private boolean moveToCoordinates(double targetX, double targetY) {
    double angleToTarget = Math.atan2(targetY - myY, targetX - myX);
    double currentHeading = getHeading();
    
    // Si l'angle actuel est différent de l'angle cible, tourner
    if (!isSameDirection(currentHeading, angleToTarget)) {
      if (normalizeAngle(angleToTarget - currentHeading) < 0) {
        stepTurn(Parameters.Direction.LEFT);
      } else {
        stepTurn(Parameters.Direction.RIGHT);
      }
      return false;
    }
    // Si on est bien aligné, avancer
    else {
      myMove();
      return true;
    }
  }
  
  

  private void myMove(){
    isMoving=true;
    move();
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
  private boolean isRoughlySameDirection(double dir1, double dir2){
    // Utiliser la valeur absolue pour une comparaison plus robuste
    return Math.abs(normalizeAngle(dir1)-normalizeAngle(dir2))<FIREANGLEPRECISION;
  }

  private void firePosition(double x, double y){
    if (myX<=x) fire(Math.atan((y-myY)/(double)(x-myX)));
    else fire(Math.PI+Math.atan((y-myY)/(double)(x-myX)));
    return;
  }
  private boolean onTheWay(double angle){
    if (myX<=targetX) return isRoughlySameDirection(angle,Math.atan((targetY-myY)/(double)(targetX-myX)));
    else return isRoughlySameDirection(angle,Math.PI+Math.atan((targetY-myY)/(double)(targetX-myX)));
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

    /**
   * Sends a formatted message through broadcast
   * @param messageType Type of message (FIRE, FALLBACK, etc.)
   * @param data Array of data to include in the message
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
   * @param message Le message à traiter
   */
  private void process(String message) {
    String[] parts = message.split(":");
    int messageType = Integer.parseInt(parts[2]);
    
    switch (messageType) {
      case FIRE:
        // Traite un message de tir
        fireOrder = true;
        countDown = 0;
        targetX = Double.parseDouble(parts[3]);
        targetY = Double.parseDouble(parts[4]);
        break;
        
      case FALLBACK:
        // Traite un message de repli
        // À implémenter selon les besoins
        break;
        
      case ROGER:
        // Traite un message d'acquittement
        // Send coordinates of Beta
        sendMessage(ROGER, myX, myY);
        break;
      
      case DODGE:
        targetX = Double.parseDouble(parts[3]);
        targetY = Double.parseDouble(parts[4]);
        state = DODGE;
        break;
        
      default:
        // Message de type inconnu
        sendLogMessage("Unknown message type: " + messageType);
        break;
    }
  }

}