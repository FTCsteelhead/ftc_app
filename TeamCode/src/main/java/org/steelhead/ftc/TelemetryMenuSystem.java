package org.steelhead.ftc;

import java.util.ArrayList;

/**
 * Created by Alec Matthews on 11/21/2016.
 * This class encapsulates the while menu system into one structure
 */

public class TelemetryMenuSystem {
    private ArrayList<TelemetryMenu> menuSystem;
    private int menuPosition = 0;
    private int menuCount = 0;
    private String menuName;

    /**
     * The Menu System is all encapsulated by the TelemetryMenuSystem class. This class keeps track
     * of what menu is currently active. You can add a remove menus dynamically with the addMenu and
     * removeMenu methods.
     *
     */
    public TelemetryMenuSystem(String name) {
        this.menuName = name;
        menuSystem = new ArrayList<>();
    }
    public void updateMenuSystem() {
        //aligns menu count with the zero beginning menuPosition
        if (menuPosition <= (menuCount-1)) {
            menuSystem.get(menuPosition).updateMenu();
        }
    }
    public void advanceMenu() {
        menuPosition++;
    }
    public void goBackMenu() {
        menuPosition--;
    }
    public void goToMenu(int menuPosition) {
        this.menuPosition = menuPosition;
    }
    public void addMenu(TelemetryMenu menu) {
        if (menu != null) {
            menuSystem.add(menu);
            menuCount++;
        }
    }
    public void removeMenu(int menuPosition) {
        menuSystem.remove(menuPosition);
        menuCount--;
    }
}
