package org.steelhead.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Created by Alec Matthews on 11/19/2016.
 * This class provides a menu to be displayed with the telemetry function
 */

public class TelemetryMenu {
    private OpMode opMode;
    private Gamepad gamepad;
    private String menuTitle;
    private boolean menuActive = false;
    private ElapsedTime debounceTime = new ElapsedTime();

    private int menuPosition = 0;
    private ArrayList<MenuItem> menuItems;

    /**
     *  This class defines a menu. A menu is a collection of MenuItems. You add items with the
     *  addItem method. This class handles all of the cursor movement and selection of objects.
     *  the updateMenu method checks buttons and prints the telemetry.
     */

    public TelemetryMenu(OpMode currentOpMode, Gamepad gamepad, String menuTitle) {
        this.opMode = currentOpMode;
        this.gamepad = gamepad;
        this.menuTitle = menuTitle;
        menuItems = new ArrayList<>();
    }

    //TODO: check some of the debounce times; they seem long. Maybe change the method of debounce.
    public void updateMenu() {
        opMode.telemetry.addLine(menuTitle);
        for (MenuItem item : menuItems) {
            opMode.telemetry.addData(item.getSelected() ? "*" : " ", item.getText());
        }

        if (gamepad.dpad_down && debounceTime.milliseconds() > 400) {
            menuItems.get(menuPosition).setSelected(false);
            menuPosition++;
            if (menuPosition > (menuItems.size() - 1)) {
                menuPosition = 0;
            }
            menuItems.get(menuPosition).setSelected(true);
            debounceTime.reset();
        } else if (gamepad.dpad_up && debounceTime.milliseconds() > 400) {
            menuItems.get(menuPosition).setSelected(false);
            menuPosition--;
            if (menuPosition < 0) {
                menuPosition = (menuItems.size() - 1);
            }
            menuItems.get(menuPosition).setSelected(true);
            debounceTime.reset();
        } else if (gamepad.a && debounceTime.milliseconds() > 400) {
            menuItems.get(menuPosition).itemClicked();
            debounceTime.reset();
        }
    }

    public void addItem(MenuItem item) {
        menuItems.add(item);
    }

    @Deprecated
    public void setMenuActive(boolean menuActive) {
        this.menuActive = menuActive;
    }

    @Deprecated
    public boolean getMenuActive() {
        return menuActive;
    }
}
