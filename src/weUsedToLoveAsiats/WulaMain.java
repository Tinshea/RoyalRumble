/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

public class WulaMain extends BrainDetectTank{

    @Override
    public void step() {
//        logPosition();
        super.step();
        MasterMind.getInstance().giveMeOrderMaster(this);
    }
}
