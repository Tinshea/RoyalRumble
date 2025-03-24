/*
 * Copyright (C) by Courtanet, All Rights Reserved.
 */
package weUsedToLoveAsiats;

import java.util.ArrayList;

public class DetectBrainAffichage {

    private static DetectBrainAffichage instance = new DetectBrainAffichage();
    private ArrayList<BrainDetectScout> scouts;
    private ArrayList<BrainDetectTank> tanks;

    private DetectBrainAffichage() {
        scouts = new ArrayList<>();
        tanks = new ArrayList<>();
    }

    public static DetectBrainAffichage getInstance() {
        return instance;
    }

    public void add(BrainDetectScout bds) {
        scouts.add(bds);
    }

    public void add(BrainDetectTank bds) {
        tanks.add(bds);
    }

    public BrainDetectScout getScout(int i) {
        return scouts.get(i);
    }

    public BrainDetectTank getTank(int i) {
        return tanks.get(i);
    }

    public int getSize() {
        return scouts.size();
    }

    public void remove(int i) {
        scouts.remove(i);
    }

    public void remove(DetectBrain bds) {
        scouts.remove(bds);
    }
}
