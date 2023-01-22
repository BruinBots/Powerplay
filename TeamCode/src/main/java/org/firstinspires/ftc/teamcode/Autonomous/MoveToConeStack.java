package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.teamcode.Karen;

public class MoveToConeStack
{

    public enum Direction { Left, Right };
    private enum State { Strafing, LineFollowing };
    private enum Color { Red, Blue };

    private State currentState;
    private int maxTravelInches;
    private Direction strafeDirection;
    private Karen robot;
    private ColorSensor colorsense;
    private Color ourColor;
    private int distanceTraveled;
    private UltrasonicSensor distanceSensor;

    public MoveToConeStack(Color c, Direction dir, int maxInches, Karen bot)
    {
        currentState = State.Strafing;
        strafeDirection = dir;
        maxTravelInches = maxInches;
        robot = bot;
        colorsense = hardwareMap.colorSensor.get("colorSensor");
        ourColor = c;
        distanceTraveled = 0;
        distanceSensor = hardwareMap.ultrasonicSensor.get("distanceSensor");
    }

    // Robot should be in one of the four corner squares facing the wall.
// Algorithm will strafe left/right until line is found and then follow line to within 8 inches of the wall.
// Move method returns true while controlling robot, false when it is done.
// When false is returned the robot will be on the line and within 8 inches of the wall.
// If the distance strafed exceeds the max distance provided then the LineNotFoundException is thrown.  No
// line was encountered within the expected distance, so alternate robot guidance needs to be used.
    public boolean Move() throws LineNotFoundException
{
    if (currentState == State.Strafing)
    {
        if ((ourColor == Color.Red && colorsense.red() > 10) ||(ourColor == Color.Blue && colorsense.blue() > 10))
        {
            // stop robot strafing movement
            currentState = State.LineFollowing;
            return true;
        }
			else
        {
            // check distance strafed
            if (distanceTraveled > maxTravelInches) {
                throw new LineNotFoundException();
            }
            // continue strafing
            return true;
        }

    }
    else // LineFollowing
    {
        // get distance from wall
        double distance = distanceSensor.getUltrasonicLevel();
        if (within 8 inches of wall)
        {
            return false; // done
        }
			else
        {
// line following algorithm here!

        }
    }
}
}

class LineNotFoundException extends Exception
{
    LineNotFoundException() { }
}