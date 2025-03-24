/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats.tools;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.round;
import static java.lang.Math.sin;



public class CoordHelper {

    public static PolarCoordinate cartToPol(int origX, int origY, int toX, int toY) {
        return cartToPol(new CartCoordinate(round(origX), round(origY)), new CartCoordinate(round(toX), round(toY)));
    }

    public static PolarCoordinate cartToPol(int origX, int origY, CartCoordinate dest) {
        return cartToPol(new CartCoordinate(round(origX), round(origY)), dest);
    }

    public static PolarCoordinate cartToPol(CartCoordinate orig, int toX, int toY) {
        return cartToPol(orig, new CartCoordinate(round(toX), round(toY)));
    }

    public static PolarCoordinate cartToPol(CartCoordinate orig, CartCoordinate to) {
        double dx = to.getX() - orig.getX();
        double dy = to.getY() - orig.getY();
        double angle= atan2(dy,dx);
        return new PolarCoordinate(angle,orig.distance(to));
    }

    public static CartCoordinate polToCart(CartCoordinate orig, PolarCoordinate polCo) {
        return polToCart(orig, polCo.getAngle(), polCo.getDist());
    }

    public static CartCoordinate polToCart(CartCoordinate orig, double angle, double dist) {
        return polToCart(orig.getX(), orig.getY(), angle, dist);
    }

    public static CartCoordinate polToCart(double origX, double origY, double angle, double dist) {
        double x = origX + (dist * cos(angle));
        x = (x > 3000) ? 3000 : x;
        x = (x < 0) ? 0 : x;
        double y = origY + (dist * sin(angle));
        y = (y > 2000) ? 2000 : y;
        y = (y < 0) ? 0 : y;
        return new CartCoordinate(x, y);
    }

    public static boolean isOutOfBound(CartCoordinate coord){
        return (coord.getX() <= 100.0 || coord.getX() >= 3000.0 || coord.getY() <= 100.0 || coord.getY() >= 2000.0);
    }
}
