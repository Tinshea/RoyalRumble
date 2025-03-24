/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

import static java.lang.Math.abs;

public class WulaSecondary extends BrainDetectScout {

    @Override
    public void step() {
        super.step();
//        logPosition();


        MasterMind.getInstance().giveMeOrderMaster(this);
    }


}
