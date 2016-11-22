package org.steelhead.ftc;

/**
 * Created by Alec Matthews on 11/19/2016.
 * This class defines a menu item for the telemetry menu.
 */

public class MenuItem{
    private String text;
    private boolean isSelected;
    private MenuCallable callable;

    /**
     *  The MenuItem class defines what it means to be a menu. The menuCallable object
     *  is where the call back function is defined. The callback is called when the menu
     *  item is clicked.
     */
    public MenuItem(String text, boolean selected, MenuCallable menuCallable) {
        this.text = text;
        this.isSelected = selected;
        this.callable = menuCallable;
    }
    public void setText(String text) {
        this.text = text;
    }
    public String getText() {
        return text;
    }
    public void setSelected(boolean selected) {
        this.isSelected = selected;
    }
    public boolean getSelected() {
        return isSelected;
    }
    public void itemClicked() {
        if (callable != null) {
            callable.menuCallable();
        }
    }

}
