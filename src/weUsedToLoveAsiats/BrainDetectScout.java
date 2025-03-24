/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package weUsedToLoveAsiats;

import static weUsedToLoveAsiats.WhoAmI.SCOUT_1;
import static weUsedToLoveAsiats.WhoAmI.SCOUT_2;
import static characteristics.Parameters.*;

import java.util.ArrayList;

import characteristics.IRadarResult;
import weUsedToLoveAsiats.tools.CartCoordinate;

public abstract class BrainDetectScout extends DetectBrain {

    private double cpt = 0;

    private MasterMind mm = MasterMind.getInstance();

    public void activate() {

        moveSpeed = teamASecondaryBotSpeed;
        DetectBrainAffichage.getInstance().add(this);
        boolean haut = false;
        boolean bas = false;
        boolean gauche = false;
        boolean droite = false;

        if (getHeading() == WEST)
            droite = true;
        if (getHeading() == EAST)
            gauche = true;

        for (IRadarResult r : detectRadar()) {
            if (r.getObjectDirection() == SOUTH)
                haut = true;

            if (r.getObjectDirection() == NORTH)
                bas = true;

        }

        if (haut) {
            if (gauche) {
                name = SCOUT_1;
                teamGauche = true;
                myPosition = new CartCoordinate(teamASecondaryBot1InitX+50, teamASecondaryBot1InitY+50);
                logPosition();
            }

            if (droite) {
                name = SCOUT_1;
                teamGauche = false;
                myPosition = new CartCoordinate(teamBSecondaryBot1InitX+50, teamBSecondaryBot1InitY+50);
                logPosition();
            }
        }
        if (bas) {
            if (gauche) {
                name = SCOUT_2;
                teamGauche = true;
                myPosition = new CartCoordinate(teamASecondaryBot2InitX+50, teamASecondaryBot2InitY+50);
                logPosition();
            }
            if (droite) {
                name = SCOUT_2;
                teamGauche = false;
                myPosition = new CartCoordinate(teamBSecondaryBot2InitX+50, teamBSecondaryBot2InitY+50);
                logPosition();
            }
        }

        mm.addScout(this);
    }

    public CartCoordinate getPosition() {
        return myPosition;
    }

    public ArrayList<IRadarResult> getReport() {
        return detectRadar();
    }

    public CartCoordinate getPos() {
        return myPosition;
    }

    public WhoAmI getName() {
        return name;
    }



}
