/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

import static weUsedToLoveAsiats.StateMM.*;
import static weUsedToLoveAsiats.tools.trucsDeBinhNul.Orders.CHILL;
import static characteristics.IRadarResult.Types.OpponentMainBot;
import static characteristics.IRadarResult.Types.OpponentSecondaryBot;
import static characteristics.Parameters.Direction.LEFT;
import static characteristics.Parameters.Direction.RIGHT;
import static java.lang.Math.*;
import static java.lang.Math.PI;
import static weUsedToLoveAsiats.WhoAmI.TANK_1;

import java.util.*;
import java.util.stream.Collectors;

import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;
import weUsedToLoveAsiats.tools.*;
import weUsedToLoveAsiats.tools.trucsDeBinhNul.Orders;
import weUsedToLoveAsiats.tools.trucsDeBinhNul.Position;

//import javafx.util.Pair;

public class MasterMind {

    private boolean left;
    private static MasterMind instance = new MasterMind();
    private ArrayList<Position> targets = new ArrayList<>();
    private ArrayList<BrainDetectScout> scouts = new ArrayList<>(2);
    private ArrayList<BrainDetectTank> tanks = new ArrayList<>(3);

    private ArrayList<CartCoordinate> wrecks = new ArrayList<>();

    private int rerunAStar = 0;
    private int cptBot = 0;
    private int deadBot = 0;
    private int scout1Isdead = 0;
    private int scout2Isdead = 0;
    private int tank1Isdead = 0;
    private int tank2Isdead = 0;
    private int tank3Isdead = 0;

    private StateMM state = DEPLOY;

    private RobotInstruction scout1;
    private RobotInstruction scout2;
    private RobotInstruction tank1;
    private RobotInstruction tank2;
    private RobotInstruction tank3;

    private ArrayList<CartCoordinate> pathScout1 = new ArrayList<>();
    private ArrayList<CartCoordinate> pathScout2 = new ArrayList<>();
    private ArrayList<CartCoordinate> pathTank1 = new ArrayList<>();
    private ArrayList<CartCoordinate> pathTank2 = new ArrayList<>();
    private ArrayList<CartCoordinate> pathTank3 = new ArrayList<>();

    private Map<DetectBrain, RobotInstruction> instrucstions = new HashMap<>();

    public ArrayList<Position> getTargets() {
        return targets;
    }

    public void updateWarField() {
        ArrayList<Brain> tmp = new ArrayList<>();
        tmp.addAll(scouts);
        tmp.addAll(tanks);
        targets.clear();
        for (Brain bot : tmp) {
            ArrayList<IRadarResult> report = null;
            CartCoordinate pos = null;

            if (bot instanceof DetectBrain) {

                DetectBrain brainDetectScout = (DetectBrain) bot;
                report = brainDetectScout.getReport();
                pos = brainDetectScout.getPos();
            }

            for (IRadarResult r : report) {
                CartCoordinate rPos = CoordHelper
                        .polToCart(pos, r.getObjectDirection(), r.getObjectDistance());
                double radius = 0;
                switch (r.getObjectType()) {
                    case OpponentMainBot:
                        break;
                    case OpponentSecondaryBot:
                        break;
                    case TeamMainBot:
                        break;
                    case TeamSecondaryBot:
                        break;
                    case Wreck:
                        addWreck(rPos);
                        radius = Parameters.teamAMainBotRadius;
                        break;
                    case BULLET:
                        radius = Parameters.bulletRadius;
                        break;
                }
                targets.add(new Position(rPos.getX(), rPos.getY(), r.getObjectType()));
            }
        }
    }

    public ArrayList<CartCoordinate> fireForEffect() {
        ArrayList<CartCoordinate> ennemies = new ArrayList<>();

        double avgX = tanks.stream().mapToDouble(brain -> brain.getPos().getX()).average().orElse(0.0);
        double avgY = tanks.stream().mapToDouble(brain -> brain.getPos().getY()).average().orElse(0.0);
        CartCoordinate avg = new CartCoordinate(avgX, avgY);

        List<CartCoordinate> collect = targets.stream().filter(pos -> pos.getTypes() == OpponentMainBot)
                .map(pos -> new CartCoordinate(pos.getX(), pos.getY()))
                .collect(Collectors.toList());
        ennemies.addAll(collect);

        List<CartCoordinate> collect2 = targets.stream().filter(pos -> pos.getTypes() == OpponentSecondaryBot)
                .map(pos -> new CartCoordinate(pos.getX(), pos.getY()))
                .collect(Collectors.toList());
        ennemies.addAll(collect2);

        return ennemies;

    }

    public static MasterMind getInstance() {
        return instance;
    }

    public void addScout(BrainDetectScout scout) {
        cptBot++;
        scouts.add(scout);
        if (cptBot == 5)
            initInstructions();
    }

    private void initInstructions() {

        for (BrainDetectScout scout : scouts) {
            left = scout.isTeamGauche();
            switch (scout.getName()) {
                case SCOUT_1:
                    scout1 = new RobotInstruction(scout);
                    instrucstions.put(scout, scout1);
                    scout1.setObjective(new CartCoordinate(left ? 500 : 2500, 650));
                    break;
                case SCOUT_2:
                    scout2 = new RobotInstruction(scout);
                    scout2.setObjective(new CartCoordinate(left ? 500 : 2500, 1450));
                    break;
            }
        }
        for (BrainDetectTank tank : tanks) {
            switch (tank.getName()) {
                case TANK_1:
                    tank1 = new RobotInstruction(tank);
                    instrucstions.put(tank, tank1);
                    tank1.setObjective(new CartCoordinate(left ? 350 : 2650, 450));
                    break;
                case TANK_2:
                    tank2 = new RobotInstruction(tank);
                    instrucstions.put(tank, tank2);
                    tank2.setObjective(new CartCoordinate(left ? 100 : 2900, 1050));
                    break;
                case TANK_3:
                    tank3 = new RobotInstruction(tank);
                    instrucstions.put(tank, tank3);
                    tank3.setObjective(new CartCoordinate(left ? 350 : 2650, 1650));
                    break;
            }
        }
    }

    public void addTanks(BrainDetectTank tank) {
        cptBot++;
        tanks.add(tank);
    }

    public void step(DetectBrain slave) {
        if (slave.getName() != TANK_1)
            return;

        updateWarField();

    }

    private boolean shouldNotFire(BrainDetectTank tank, CartCoordinate target) {
        PolarCoordinate pc = CoordHelper.cartToPol(tank.getPos(), target);

        if (pc.getDist() > 1000) {
            return true;
        }
        return shouldNotFire(tank, pc.getAngle());
    }

    private boolean shouldNotFire(BrainDetectTank tank, double angle) {
        ArrayList<DetectBrain> allBot = new ArrayList<>();
        allBot.addAll(scouts);
        allBot.addAll(tanks);

        for (DetectBrain detectBrain : allBot) {
            double x = detectBrain.getPos().getX();
            double y = detectBrain.getPos().getY();

            PolarCoordinate bd = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x + 60, y + 60));
            PolarCoordinate hg = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x - 60, y - 60));
            PolarCoordinate bg = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x - 60, y + 60));
            PolarCoordinate hd = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x + 60, y - 60));

            if (isBetweenAngle(angle, hg.getAngle(), bd.getAngle()) || isBetweenAngle(angle, bg
                    .getAngle(), hd.getAngle())) {
                return true;
            }
        }
        for (CartCoordinate wreck : wrecks) {
            double x = wreck.getX();
            double y = wreck.getY();

            PolarCoordinate bd = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x + 60, y + 60));
            PolarCoordinate hg = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x - 60, y - 60));
            PolarCoordinate bg = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x - 60, y + 60));
            PolarCoordinate hd = CoordHelper.cartToPol(tank.getPos(), new CartCoordinate(x + 60, y - 60));

            if (isBetweenAngle(angle, hg.getAngle(), bd.getAngle()) || isBetweenAngle(angle, bg
                    .getAngle(), hd.getAngle())) {
                return true;
            }
        }
        return false;
    }

    private double normalizeAngle(double angle) {
        while (angle < -PI)
            angle += 2 * PI;
        while (angle > PI)
            angle -= 2 * PI;
        return angle;
    }

    private boolean isBetweenAngle(double angle, double a, double b) {
        a -= angle;
        b -= angle;
        normalizeAngle(a);
        normalizeAngle(b);
        if (a * b >= 0) {
            return false;
        } else
            return Math.abs(a - b) < PI;
    }

    private boolean done() {
        return scout1.isDone() && scout2.isDone() && tank1.isDone() && tank2.isDone() && tank3.isDone();
    }

    private ArrayList<CartCoordinate> findPath(CartCoordinate coord, ArrayList<CartCoordinate> wrecks) {
        System.out.println("find a path");
        CartCoordinate UpLeft = new CartCoordinate(200, 200);
        CartCoordinate underUpLeft = new CartCoordinate(400, 1000);
        CartCoordinate righterUpLeft = new CartCoordinate(1200, 200);
        CartCoordinate DownRight = new CartCoordinate(2600, 1800);
        CartCoordinate upperDownRight = new CartCoordinate(2800, 1000);
        ArrayList<CartCoordinate> res = new ArrayList<>();

        res.addAll(Astar.aStar(coord, underUpLeft, wrecks));
        res.addAll(Astar.aStar(underUpLeft, UpLeft, wrecks));
        res.addAll(Astar.aStar(UpLeft, righterUpLeft, wrecks));
        res.addAll(Astar.aStar(righterUpLeft, DownRight, wrecks));
        res.addAll(Astar.aStar(DownRight, upperDownRight, wrecks));
        return res;
    }

    private Orders turnAround(ArrayList<CartCoordinate> path, DetectBrain slave, RobotInstruction robot) {
//        if (otherBotNear(slave)) {
//            System.out.println("proche!");
//            ArrayList<DetectBrain> bots = new ArrayList<>();
//            bots.addAll(tanks);
//            bots.addAll(scouts);
//            bots.remove(slave);
//            ArrayList<CartCoordinate> newWreck = new ArrayList<>(wrecks);
//            newWreck.addAll(bots.stream().map(DetectBrain::getPos).collect(Collectors.toList()));
//            path.addAll(0, Astar.aStar(slave.getPos(), robot.getObjective(), newWreck));
//        }
        if (robot.isDone()) path.remove(0);
        if (path.isEmpty()) return robot.getCurrentOrder();
        slave.sendLogMessage("go to " + path.get(0));
        robot.setObjective(path.get(0));
        robot.majObj();
        return robot.getCurrentOrder();
    }

    private void seekAndDestroyPath() {
        rerunAStar++;
        System.out.println("re find path");
        scout1.setSpeedOf10(10);
        scout2.setSpeedOf10(10);
        if (left) {
            pathScout1 = Astar.aStar(scout1.getPosRobot(), new CartCoordinate(left ? 2900 : 2000, 600), wrecks);
            pathScout2 = Astar.aStar(scout2.getPosRobot(), new CartCoordinate(left ? 2900 : 2000, 1200), wrecks);
            pathTank1 = Astar.aStar(tank1.getPosRobot(), new CartCoordinate(left ? 2900 : 2000, 400), wrecks);
            pathTank2 = Astar.aStar(tank2.getPosRobot(), new CartCoordinate(left ? 2900 : 2000, 1100), wrecks);
            pathTank3 = Astar.aStar(tank3.getPosRobot(), new CartCoordinate(left ? 2900 : 2000, 1700), wrecks);
        } else {
            pathScout1.add(new CartCoordinate( 2000, 600));
            pathScout2.add(new CartCoordinate( 2000, 1200));
            pathTank1.add(new CartCoordinate( 2000, 400));
            pathTank2.add(new CartCoordinate( 2000, 1100));
            pathTank3.add(new CartCoordinate( 2000, 1700));

        }

    }

    private boolean otherBotNear(DetectBrain slave) {
        ArrayList<DetectBrain> bots = new ArrayList<>();
        bots.addAll(tanks);
        bots.addAll(scouts);
        for (DetectBrain bot : bots) {
            if (bot == slave) continue;
            if (bot.getPos().distance(slave.getPos()) < 300) {
                return true;
            }
        }
        return false;
    }

    public void giveMeOrderMaster(DetectBrain slave) {
        if (state == DEPLOY && !left) state = CHARGE;
        deadBot = tank1Isdead + tank2Isdead + tank3Isdead + scout1Isdead + scout2Isdead;
        Orders currentOrder = CHILL;
        boolean wall = scout1.getPosRobot().getX() > 2800 || scout2.getPosRobot().getX() > 2800 || tank1.getPosRobot().getX() > 2800 || tank2.getPosRobot().getX() > 2800 || tank3.getPosRobot().getX() > 2800;
        if (wrecks.size() >= 5 + deadBot && left || wall) {
            if (state != TURN) {
                System.out.println(TURN);
                state = TURN;
                pathTank1 = new ArrayList<>();
                pathTank2 = new ArrayList<>();
                pathTank3 = new ArrayList<>();
                pathScout1 = new ArrayList<>();
                pathScout2 = new ArrayList<>();
                scout1.setSpeedOf10(4);
                scout2.setSpeedOf10(4);
                tank1.setSpeedOf10(10);
                tank2.setSpeedOf10(10);
                tank3.setSpeedOf10(9);
            }
        }
        if (state == TURN) {
            switch (slave.getName()) {
                case TANK_1:
                    if (pathTank1 == null || pathTank1.isEmpty()) pathTank1 = findPath(slave.getPos(), wrecks);
                    currentOrder = turnAround(pathTank1, slave, tank1);
                    break;
                case TANK_2:
                    if (pathTank2 == null || pathTank2.isEmpty()) pathTank2 = findPath(slave.getPos(), wrecks);
                    currentOrder = turnAround(pathTank2, slave, tank2);
                    break;
                case TANK_3:
                    if (pathTank3 == null || pathTank3.isEmpty()) pathTank3 = findPath(slave.getPos(), wrecks);
                    currentOrder = turnAround(pathTank3, slave, tank3);
                    break;
                case SCOUT_1:
                    if (pathScout1 == null || pathScout1.isEmpty()) pathScout1 = findPath(slave.getPos(), wrecks);
                    currentOrder = turnAround(pathScout1, slave, scout1);
                    break;
                case SCOUT_2:
                    if (pathScout2 == null || pathScout2.isEmpty()) pathScout2 = findPath(slave.getPos(), wrecks);
                    currentOrder = turnAround(pathScout2, slave, scout2);
                    break;
            }
        }
        boolean t1Ashot = false;
        boolean t2Ashot = false;
        boolean t3Ashot = false;
        boolean asShot = false;
        ArrayList<CartCoordinate> targets = fireForEffect();
        if (!targets.isEmpty()) {
            asShot = true;
            boolean tire1 = fire(targets.get(0), tank1.getMyBot());
            boolean tire2 = fire(targets.get(0), tank2.getMyBot());
            boolean tire3 = fire(targets.get(0), tank3.getMyBot());
            for (CartCoordinate target : targets) {
                if (tire1 && tire2 && tire3)
                    break;
                if (!tire1)
                    tire1 = fire(target, tank1.getMyBot());
                if (!tire2)
                    tire2 = fire(target, tank2.getMyBot());
                if (!tire3)
                    tire3 = fire(target, tank3.getMyBot());
            }
            t1Ashot = tire1;
            t2Ashot = tire2;
            t3Ashot = tire3;
        }

        if (slave.getName() == TANK_1 && state == DEPLOY) {

            if (scout1.isDone() && state == DEPLOY && tank1.isDone())
                scout1.setObjective(new CartCoordinate(left ? 400 : 2600, 650));

            if (scout2.isDone() && state == DEPLOY && tank3.isDone())
                scout2.setObjective(new CartCoordinate(left ? 400 : 2600, 1450));

            if (done() && state == DEPLOY) {
                state = CHARGE;
                System.out.println(CHARGE);
                seekAndDestroyPath();
            }

        }
        if (state != TURN) {
            switch (slave.getName()) {

                case TANK_1:
                    if (slave.getHealth() <= 0) {
                        tank1Isdead = 1;
                        addWreck(slave.getPos());
                    }
                    if (tank1.isDone() && !pathTank1.isEmpty()) tank1.setObjective(pathTank1.remove(0));
                    tank1.majObj();
                    if (!t1Ashot) {
                        fire(tank1.fire(slave), slave);
                        currentOrder = tank1.getCurrentOrder();
                    }
                    slave.sendLogMessage(tank1.isDone() + " go to " + tank1.getObjective() + " " + wrecks.size() + "-" + deadBot);
                    break;
                case TANK_2:
                    if (slave.getHealth() <= 0) {
                        tank2Isdead = 1;
                        addWreck(slave.getPos());
                    }
                    if (tank2.isDone() && !pathTank2.isEmpty()) tank2.setObjective(pathTank2.remove(0));
                    tank2.majObj();
                    if (!t2Ashot) {
                        fire(tank2.fire(slave), slave);
                        currentOrder = tank2.getCurrentOrder();
                    }
                    slave.sendLogMessage(tank2.isDone() + " go to " + tank2.getObjective());
                    break;
                case TANK_3:
                    if (slave.getHealth() <= 0) {
                        tank2Isdead = 1;
                        addWreck(slave.getPos());
                    }
                    if (tank3.isDone() && !pathTank3.isEmpty()) tank3.setObjective(pathTank3.remove(0));
                    tank3.majObj();
                    if (!t3Ashot) {
                        fire(tank3.fire(slave), slave);
                        currentOrder = tank3.getCurrentOrder();
                    }
                    slave.sendLogMessage(tank3.isDone() + " go to " + tank3.getObjective());
                    break;
                case SCOUT_1:
                    if (slave.getHealth() <= 0) {
                        scout1Isdead = 1;
                        addWreck(slave.getPos());
                    }
                    if (scout1.isDone() && !pathScout1.isEmpty()) scout1.setObjective(pathScout1.remove(0));
                    scout1.majObj();
                    if (!t1Ashot)
                        currentOrder = scout1.getCurrentOrder();
                    slave.sendLogMessage(scout1.isDone() + " go to " + scout1.getObjective() + " " + slave.getHealth());
                    break;
                case SCOUT_2:
                    if (slave.getHealth() <= 0) {
                        scout2Isdead = 1;
                        addWreck(slave.getPos());
                    }
                    if (scout2.isDone() && !pathScout2.isEmpty()) scout2.setObjective(pathScout2.remove(0));
                    scout2.majObj();
                    if (!t3Ashot)
                        currentOrder = scout2.getCurrentOrder();
                    slave.sendLogMessage(scout2.isDone() + " go to " + scout2.getObjective() + " " + slave.getHealth());
                    break;
            }
        }
        switch (currentOrder) {
            case MOVE:
                slave.move();
                break;
            case MOVEBACK:
                slave.moveBack();
                break;
            case TURNRIGHT:
                slave.stepTurn(RIGHT);
                break;
            case TURNLEFT:
                slave.stepTurn(LEFT);
                break;
            case CHILL:
                break;

        }
    }

    private void addWreck(CartCoordinate pos) {
        for (CartCoordinate wreck : wrecks) {
            if (abs(wreck.getX() - pos.getX()) < 1 && abs(wreck.getY() - pos.getY()) < 1)
                return;
        }
        wrecks.add(pos);
        seekAndDestroyPath();
    }

    private boolean fire(double angle, DetectBrain bot) {
        BrainDetectTank tank = (BrainDetectTank) bot;
        if (!shouldNotFire(tank, angle)) {
            bot.fire(angle);
            return true;
        }
        return false;
    }

    private boolean fire(CartCoordinate target, DetectBrain bot) {
        BrainDetectTank tank = (BrainDetectTank) bot;
        if (!shouldNotFire(tank, target)) {
            instrucstions.get(bot).fire(target);
            return true;
        }
        return false;
    }

    public void setLeft(boolean left) {
        this.left = left;
    }

    public boolean isLeft() {
        return left;

    }
}
