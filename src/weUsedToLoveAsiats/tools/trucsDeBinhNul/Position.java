/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats.tools.trucsDeBinhNul;

import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.RadarResult;

public class Position {
    private double x;
    private double y;
    private RadarResult.Types types;

    public Position(double x, double y) {
        this.x = x;
        this.y = y;
        types = null;
    }

    public Position(double x, double y, RadarResult.Types types) {
        this.x = x;
        this.y = y;
        this.types = types;
    }

    public void setTypes(RadarResult.Types types) {
        this.types = types;
    }

    public RadarResult.Types getTypes() {
        return types;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
