/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

import static weUsedToLoveAsiats.tools.CoordHelper.isOutOfBound;
import static weUsedToLoveAsiats.tools.CoordHelper.polToCart;

import java.util.ArrayList;

import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;
import weUsedToLoveAsiats.tools.CartCoordinate;

public abstract class DetectBrain extends Brain {

    protected CartCoordinate myPosition;
    protected WhoAmI name;
    protected double moveSpeed;
    protected boolean teamGauche;

    public WhoAmI getName() {
        return name;
    }

    public abstract CartCoordinate getPos();

    public abstract ArrayList<IRadarResult> getReport();

    protected void logPosition() {
        sendLogMessage(name + " x:" + myPosition.getX() + " y:" + myPosition.getY());
    }

    public void move(boolean back) {
        double speed = back ? -moveSpeed : moveSpeed;
        boolean collision = false;
        CartCoordinate newCartCoordinate = polToCart(myPosition, getHeading(), speed);
        boolean bull = false;
        for (IRadarResult r : detectRadar()) {
            double d = Double.MAX_VALUE;
            switch (r.getObjectType()) {
                case OpponentMainBot:
                case OpponentSecondaryBot:
                case TeamMainBot:
                case TeamSecondaryBot:
                case Wreck:
                    break;
                case BULLET:
                    bull = true;
                    break;
            }
            CartCoordinate botPos = polToCart(myPosition, r.getObjectDirection(), r.getObjectDistance());
            if (!bull)
                if (!collision)
                    collision = (newCartCoordinate.getX() - botPos.getX()) * (newCartCoordinate.getX() - botPos
                                    .getX()) + (newCartCoordinate.getY() - botPos
                                    .getY()) * (newCartCoordinate.getY() - botPos
                                    .getY()) < (Parameters.teamAMainBotRadius + 50) * (Parameters.teamAMainBotRadius
                                    + 50);
        }

        if (!isOutOfBound(newCartCoordinate)) {
            if (!collision && getHealth() > 0) {
                myPosition = newCartCoordinate;
            }
        }

//            logPosition();
        if (back)
            super.moveBack();
        else
            super.move();

    }

    @Override
    public void step() {
        MasterMind.getInstance().step(this);
    }

    @Override
    public void move() {
        move(false);
    }

    @Override
    public void moveBack() {
        move(true);
    }

    public boolean isTeamGauche() {
        return teamGauche;
    }
}
