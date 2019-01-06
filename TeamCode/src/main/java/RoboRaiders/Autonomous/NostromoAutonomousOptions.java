package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBot;

@Autonomous
@Disabled

public class NostromoAutonomousOptions extends NostromoAutonomousMethods {

    public NostromoBot robot = new NostromoBot();

    boolean cur_B_ButtonState;                                            // "b" button current state
    boolean cur_X_ButtonState;                                            // "x" button current state

    boolean prev_B_ButtonState;                                           // "b" button previous state
    boolean prev_X_ButtonState;                                           // "x" button previous state

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Array to store selected options, by design selectedOptions[0][*] is used to store the
        // response for finalizing the selected options.
        String[][] selectedOptions = new String[8][2];                    // Array to store selections

        String allianceSelPrompt = "Alliance Color";                      // Alliance Color prompt
        final String[] alliancePosResps = new String[]                    // Possible Alliance Color selections
                {"Red", "Blue"};

        String locationSelPrompt = "Location?";                           // Robots Starting Position prompt
        final String[] locationPosResps = new String[]                    // Possible Starting Positions selections
                {"Depot", "Crater"};

        String deployRobotSelPrompt = "Deploy From Lander?";              // Deploy  prompt
        final String[] deployRobotPosResps = new String[]                 // Possible Deploy selections
                {"No", "Yes"};

        String dropTeamMarkerSelPrompt = "Drop Team Marker?";             // Drop team marker prompt
        final String[] dropTeamMarkerPosResps = new String[]              // Possible drop team marker selections
                {"No", "Yes"};
        String moveDepotSelPrompt = "Move to Depot?";                     // Move to depot prompt
        final String [] moveDepotPosResps = new String[]                  // Possible move to depot selections
                {"No, Yes"};
        String parkSelPrompt = "Park" ;                                   // Parking prompt
        final String [] parkPosResps = new String[]                       // Possible Parking selections
                {"No, Yes"};

        String selectionsOk = "Selections Great :)";                      // Finished Selections prompt
        final String[] finishedSelPosResps = new String[]                 // Possible Finished Selections responses
                {"No", "Yes"};

        // Default that the options to be selected are not finalized.  This is done since we want
        // the loop below to actually loop.  At the end of the loop, the question will be asked if
        // the selections are ok, if they are, then this will be set to "yes" and the loop will
        // exit.
        selectedOptions[0][1] = "No";

        // Configure for Indie autonomous while the selections are not finalized
        while (selectedOptions[0][1].equals("No")) {

            //                        Prompt            Responses    Index  Options output
            configForAuto2Options(allianceSelPrompt, alliancePosResps, 1, selectedOptions);                   // Alliance Color selection
            configForAuto2Options(locationSelPrompt, locationPosResps, 2, selectedOptions);                   // Robot Starting Position selection
            configForAuto2Options(deployRobotSelPrompt, deployRobotPosResps, 3, selectedOptions);             // Deploy Robot selection
            configForAuto2Options(dropTeamMarkerSelPrompt, dropTeamMarkerPosResps, 4, selectedOptions);       // Drop team marker selection
            configForAuto2Options(moveDepotSelPrompt, moveDepotPosResps, 5, selectedOptions);                 // Move to Depot selection
            configForAuto2Options(parkSelPrompt, parkPosResps,6, selectedOptions);                            // Parking selection

            // Loop through all of the selections and tell user what s/he has selected
            for (int i = 1; i <= 6; i++) {

                telemetry.addLine().addData(selectedOptions[i][0], selectedOptions[i][1]);
            }

            // Are you sure about the options selected?
            //                        Prompt       Responses       Index Options output
            configForAuto2Options(selectionsOk, finishedSelPosResps, 0, selectedOptions);       // Finished Selections
        }

        robot.initialize(hardwareMap);

        gamepad1.reset();

        while (!gamepad1.b) {

        }

        telemetry.addLine("Initialized");
        telemetry.update();

        // Wait for start to be pushed
        waitForStart();

        // Deploy From Lander
        if (selectedOptions[3][1].equals("Yes")) {
            DeployRobot(robot);
        }

        // Move to Depot
        if (selectedOptions[5][1].equals("Yes")) {
            if (selectedOptions[2][1].equals("Crater")) {
                moveDepotFromCraterStart(robot);
            }
            if (selectedOptions[2][1].equals("Depot")) {
                moveDepotFromDepotStart(robot);
            }

        // Drop Team Marker
        if (selectedOptions[4][1].equals("Yes")) {
            DeployTeamMarker(robot);
        }

        //Park In the Crater
        if (selectedOptions[6][1].equals("Yes") ) {
            if (selectedOptions[2][1].equals("Crater")) {
                parkFromCraterStart(robot);
                }
            if (selectedOptions[2][1].equals("Depot")) {
                parkFromDepotStart(robot);
               }
            }



        } else if (selectedOptions[4][1].equals("Cryptobox Vuforia")) {

            //getRelicRecoveryVuMark();
            Thread.sleep(250);

            //selectColumn(robot, selectedOptions[1][1], selectedOptions[2][1], pictograph);
        }
    }

    /**
     * configForAuto2Options will save the response (selOptions) from a set of 2 possible responses (posResps)
     * a given prompt (selPrompt)
     *
     * @param selPrompt The given configuration prompt
     * @param posResps The possible responses to a given configuration prompt
     * @param selIndex The position within selOptions array in which to store the responses
     * @param selOptions Where to store the configuration prompt and response
     */
    public void configForAuto2Options(String selPrompt, String[] posResps, int selIndex, String[][] selOptions) {

        // Let the user Select
        gamepad1.reset();

        prev_B_ButtonState = false;
        prev_X_ButtonState = false;
        cur_B_ButtonState = false;
        cur_X_ButtonState = false;

        selOptions[selIndex][0] = selPrompt;

        // Prompt User for Selection
        telemetry.addLine(selPrompt);
        telemetry.addLine(posResps[1] + " - X " + " or " + posResps[0] + " - B" );
        telemetry.update();

        // Loop until either the "b" button or the "x" button is pressed, initially we set both
        // buttons to indicate they have not been pressed.
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

        while (!(prev_B_ButtonState | prev_X_ButtonState)) {

            cur_B_ButtonState = gamepad1.b;                           // get the current state of button b
            cur_X_ButtonState = gamepad1.x;                           // get the current state of button x

            if (cur_B_ButtonState) {                                  // when the "b" button on the gamepad is pressed
                if (!prev_B_ButtonState) {                            // when the previous "b" button was NOT pushed
                    selOptions[selIndex][1] = posResps[0];            // first response was selected, store the response
                    prev_B_ButtonState = true;                        // indicate that the previous B button state is PUSHED
                }
            }
            else if (cur_X_ButtonState) {                             // when the "x" button on the gamepad is pressed
                if (!prev_X_ButtonState) {                            // when the previous "x" button was NOT pushed
                    selOptions[selIndex][1] = posResps[1];            // second response was selected, store the response
                    prev_X_ButtonState = true;                        // indicate that the previous X button state is PUSHED
                }
            }
        }

        telemetry.addLine().addData(selOptions[selIndex][0], selOptions[selIndex][1]);
        telemetry.update();

        // Wait one second
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}