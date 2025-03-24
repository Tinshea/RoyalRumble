/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

import static weUsedToLoveAsiats.tools.trucsDeBinhNul.Orders.*;
import static characteristics.Parameters.teamBSecondaryBotStepTurnAngle;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static weUsedToLoveAsiats.tools.CoordHelper.cartToPol;

import weUsedToLoveAsiats.tools.trucsDeBinhNul.Orders;
import weUsedToLoveAsiats.tools.*;

public class RobotInstruction {

    private PolarCoordinate polarBear;
    private CartCoordinate objective;
    private Orders currentOrder = CHILL;
    private boolean done = false;
    private int speedOf10 = 10;
    private int cptSpeed = 0;
    private SprayManager sp ;
    private static final double delta = 0.1;
    public int cptStop = 0;

    public DetectBrain getMyBot() {
        return myBot;
    }

    private DetectBrain myBot;

    public RobotInstruction(DetectBrain myBot) {
        this.myBot = myBot;
        setObjective(myBot.getPos());
        sp = new SprayManager(myBot.isTeamGauche()?0:PI, 0.01);
    }

    public boolean isDone() {
        return done;
    }

    public PolarCoordinate getPolarBear() {
        return polarBear;
    }

    public CartCoordinate getObjective() {
        return objective;
    }

    public Orders getCurrentOrder() {
        return currentOrder;
    }

    public CartCoordinate getPosRobot() {
        return myBot.getPos();
    }

    private double getHeading() {
        return myBot.getHeading();
    }

    public void setObjective(CartCoordinate objective) {
        this.objective = objective;
        done = false;
        majObj();
    }

    public void majObj() {
        polarBear = cartToPol(getPosRobot(), objective);
        if (polarBear.getDist() > 10) {
            if (isHeading(polarBear.getAngle())) {
                cptSpeed++;
                if (cptSpeed < speedOf10)
                    currentOrder = MOVE;
                else
                    currentOrder = CHILL;

            } else if (isLeCul(polarBear.getAngle())) {
                cptSpeed++;
                if (cptSpeed < speedOf10)
                    currentOrder = MOVEBACK;
                else
                    currentOrder = CHILL;

            } else {currentOrder = leftOrRight(polarBear.getAngle());}
        } else {
            currentOrder = CHILL;
            done = true;
        }
        if (cptSpeed == 10)
            cptSpeed = 0;
    }

    public boolean isHeading(double dir) {
        double cosDir = cos(dir);
        double sinDir = sin(dir);
        double cosBot = cos(getHeading());
        double sinBot = sin(getHeading());

        return abs(cosDir - cosBot) < cos(teamBSecondaryBotStepTurnAngle) && abs(sinDir - sinBot) <
                        sin(teamBSecondaryBotStepTurnAngle);
    }

    public boolean isHeading() {
        return isHeading(polarBear.getAngle());
    }

    public boolean isLeCul(double dir) {
        double cosDir = cos(dir);
        double sinDir = sin(dir);
        double cosBot = -cos(getHeading());
        double sinBot = -sin(getHeading());
        return abs(cosDir - cosBot) < cos(teamBSecondaryBotStepTurnAngle) && abs(sinDir - sinBot) <
                        sin(teamBSecondaryBotStepTurnAngle);
    }

    public boolean isLeCul() {
        return isLeCul(polarBear.getAngle());
    }

    public int getSpeedOf10() {
        return speedOf10;
    }

    public void setSpeedOf10(int speedOf10) {
        this.speedOf10 = speedOf10;
    }

    private Orders leftOrRight(double dir) {
        double cos = cos(getHeading() - dir);
        double sin = sin(getHeading() - dir);
        if (cos > 0) {
            if (sin > 0)
                return TURNRIGHT;
            else
                return TURNLEFT;
        } else if (sin < 0)
            return TURNLEFT;
        else
            return TURNRIGHT;

    }

    public double fire(DetectBrain slave) {
        double current = sp.getCurrent();
        return current;
    }

    public void fire(CartCoordinate target) {
        myBot.sendLogMessage(myBot.getName() + " fire at " + target);
        //TODO deal with friendly fire
        double dir = CoordHelper.cartToPol(myBot.getPos(), target).getAngle();
        myBot.fire(dir);
    }
}

