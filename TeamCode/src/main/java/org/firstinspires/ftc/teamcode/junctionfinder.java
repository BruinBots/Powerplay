/**
 go straight
 if left switch is pressed, move right
 if right switch is pressed, move left
 if left and right switches are pressed, stop!
 */

public class Main {
    static void checkstuff() {
        while true do {
            driveStraight(distance)
            if (leftSwitch.pressed) {
                moveLeft(distance)
            }
            if (rightSwitch.pressed) {
                moveRight(distance)
            }
            if (leftSwitch.pressed && rightSwitch.pressed) {
                break
            }
        }
    }
}