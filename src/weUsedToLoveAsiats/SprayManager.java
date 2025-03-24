/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

import static java.lang.Math.PI;

public class SprayManager {

    private double current, step, init;
    private boolean sens = true;
    private final double angle = PI / 12;

    public SprayManager(double current, double step) {
        this.current = current;
        this.init = current;
        this.step = step;
    }

    public double getCurrent() {
        current = sens ? current + step : current - step;
        if (sens) {
            if (current > init + angle) {
                sens = false;
            }
        } else {
            if (current < init - angle) {
                sens = true;
            }
        }
        return current;
    }

    public double getAngle() {
        return current;
    }
}
