/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package weUsedToLoveAsiats;

import static weUsedToLoveAsiats.WhoAmI.TANK_1;
import static weUsedToLoveAsiats.WhoAmI.TANK_2;
import static weUsedToLoveAsiats.WhoAmI.TANK_3;
import static characteristics.Parameters.*;

import java.util.ArrayList;

import characteristics.IRadarResult;
import characteristics.Parameters;
import weUsedToLoveAsiats.tools.CartCoordinate;

public abstract class BrainDetectTank extends DetectBrain {

    private boolean first = false;

    public void activate() {
        moveSpeed = teamAMainBotSpeed;

        int auDessus = 0;
        int enDessous = 0;
        boolean gauche = false;
        boolean droite = false;

        DetectBrainAffichage.getInstance().add(this);
        if (getHeading() == WEST)
            droite = true;
        if (getHeading() == EAST)
            gauche = true;

        for (IRadarResult r : detectRadar()) {
            if (r.getObjectDirection() == SOUTH)
                enDessous++;
            if (r.getObjectDirection() == NORTH)
                auDessus++;
        }

        if (auDessus == 0) {
            first = true;
            if (gauche) {
                name = TANK_1;
                teamGauche = true;
                myPosition = new CartCoordinate(Parameters.teamAMainBot1InitX + 50, Parameters.teamAMainBot1InitY + 50);
            }
            if (droite) {
                name = TANK_1;
                teamGauche = false;
                myPosition = new CartCoordinate(Parameters.teamBMainBot1InitX + 50, Parameters.teamBMainBot1InitY + 50);
            }
        }
        if (auDessus == 1 && enDessous == 1) {
            if (gauche) {
                name = TANK_2;
                teamGauche = true;
                myPosition = new CartCoordinate(Parameters.teamAMainBot2InitX + 50, Parameters.teamAMainBot2InitY + 50);
            }
            if (droite) {
                name = TANK_2;
                teamGauche = false;
                myPosition = new CartCoordinate(Parameters.teamBMainBot2InitX + 50, Parameters.teamBMainBot2InitY + 50);
            }
        }
        if (enDessous == 0) {
            if (gauche) {
                name = TANK_3;
                teamGauche = true;
                myPosition = new CartCoordinate(Parameters.teamAMainBot3InitX + 50, Parameters.teamAMainBot3InitY + 50);
            }
            if (droite) {
                name = TANK_3;
                teamGauche = false;
                myPosition = new CartCoordinate(Parameters.teamBMainBot3InitX + 50, Parameters.teamBMainBot3InitY + 50);
            }
        }
        MasterMind.getInstance().addTanks(this);
    }

    @Override
    public void step() {
        if (first)
            MasterMind.getInstance().updateWarField();
    }

    @Override
    public CartCoordinate getPos() {
        return myPosition;
    }

    @Override
    public ArrayList<IRadarResult> getReport() {
        return detectRadar();
    }

}
