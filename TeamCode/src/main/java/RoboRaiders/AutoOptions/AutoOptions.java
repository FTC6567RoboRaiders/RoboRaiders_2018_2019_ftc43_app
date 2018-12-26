package RoboRaiders.AutoOptions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.Array;
import java.util.ArrayList;


public class AutoOptions {

    private LinearOpMode op;

    private boolean prev_B_ButtonState;                                // "b" button previous state
    private boolean prev_X_ButtonState;                                // "x" button previous state

    /**
     * Constructor
     * @param op - the linear opmode tied to this class
     */
    public AutoOptions(LinearOpMode op) {this.op = op;}

    /**
     * will return the alliance selection
     * @return a boolean indicating the alliance selection, when:
     *         RED  - true
     *         BLUE - false
     */
    public boolean selectAlliance() {
        String[] alliances = new String[] {"Red", "Blue"};             // Create the selections

        // Let driver make selection, when index = 0 means first selection, when index = 1
        // means second selection
        int index = makeSelection("Alliance: ",alliances);

        // Check index against zero, if zero then true returned else false
        return index == 0;
    }

    /**
     * will return the location selection
     * @return a boolean indicating the alliance selection, when:
     *          DEPOT  - true
     *          CRATER - false
     */
    public boolean selectLocation() {

        // Create the locations
        String[] locations = new String[] {"Depot", "Crater"};

        // Let driver make selection, when index = 0 means first selection, when index = 1
        // means second selection
        int index = makeSelection("Location: ", locations);

        // Check index against zero, if zero then true returned else false
        return index == 0;
    }

    /**
     * will return yes or no if the robot should be deployed (lowered) from the lander
     * @return a boolean indicating if the robot should be deployed from the lander
     *         YES - true
     *         NO  - false
     */
    public boolean selectDeployFromLander() {

        // Let the driver make a yes or no selection for deploying from lander
        int index = makeYesNoSelection("Deploy From Lander");

        // Check index against zero, if zero, then true returned, else false
        return index == 0;

    }

    /**
     * will return yes or no if the autonomous selections are good
     * @return a boolean indicating if the autonomous selections are good
     *         YES - true
     *         NO  - false
     */
    public boolean selectionsGood() {

        // Let the driver make a yes or no selection for deploying from lander
        int index = makeYesNoSelection("Are selections good");

        // Check index against zero, if zero, then true returned, else false
        return index == 0;

    }

    /**
     * process a yes/no prompt
     * @param msYNPrompt - the prompt for the Yes/No question
     * @return index of the selection, when
     *         Yes - index = 0
     *         No  - index = 1
     */
    public int makeYesNoSelection(String msYNPrompt) {

        String[] yesNo = new String[] { "Yes", "No"};
        int index = makeSelection(msYNPrompt,yesNo);
        return index;
    }

        /**
         * will save the response (selOptions) from a set of 2 possible responses (posResps)
         * a given prompt (selPrompt)
         *
         * @param msPrompt The given configuration prompt
         * @param msResps The possible responses to a given configuration prompt
         *
         *
         */

    private int makeSelection(String msPrompt, String[] msResps) {


        int index = 0;


        // Let the user Select, reset gamepad1
        op.gamepad1.reset();

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;


        // Prompt User for Selection
        op.telemetry.addLine(msPrompt);
        op.telemetry.addLine(msResps[0] + " - X " + " or " + msResps[1] + " - B" );
        op.telemetry.update();

        // Loop until either the "b" button or the "x" button is pressed, initially we assume that
        // both buttons have not been pressed.
        //
        // The logic here says OR the previous button states and when they are both false continue
        // here is a table of how this works
        //    +--------------------+------+--------------------+--------+-------------+
        //    | prev_B_ButtonState | -OR- | prev_X_ButtonState | Result | Neg. Result |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      FALSE         | -OR- |     FALSE          | FALSE  |   TRUE      |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      FALSE         | -OR- |     TRUE           | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      TRUE          | -OR- |     FALSE          | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+
        //    |      TRUE          | -OR- |     TRUE           | TRUE   |   FALSE     |
        //    +--------------------+------+--------------------+--------+-------------+

        while (!(prev_X_ButtonState | prev_B_ButtonState)) {

            // When the X button has been pushed AND the X button was not pushed before
            if (op.gamepad1.x && !prev_X_ButtonState) {
                index = 0;                                            // first response was selected, store the response
                prev_X_ButtonState = true;                            // indicate that the X button state has been PUSHED
            }

            // When the B button has been pushed AND the B button was not pushed before
            else if (op.gamepad1.b && !prev_B_ButtonState) {
                index = 1;                                            // second response was selected, store the response
                prev_B_ButtonState = true;                            // indicate that the B button state has been PUSHED
            }
        }

        // Wait one second
        try {Thread.sleep(750);} catch (InterruptedException e) {e.printStackTrace();}

        return index;
    }

}

