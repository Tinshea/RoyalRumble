/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats.tools;

import static java.lang.Math.sqrt;

public class CartCoordinate {
    double x;
    double y;

    public CartCoordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public double distance(CartCoordinate cc) {
        return sqrt((cc.getX() - x) * (cc.getX() - x) + (cc.getY() - y) * (cc.getY() - y));
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double dotProduct(CartCoordinate q) { return this.x * q.getX() + this.y * q.getY(); }

    @Override
    public String toString() {
        return "CartCoordinate{" +
                        "x=" + x +
                        ", y=" + y +
                        '}';
    }
}
