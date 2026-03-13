package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Random;

public class LedSubsystem extends SubsystemBase {
/* 
    // ---------------- LED SETUP ----------------

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private static final int WIDTH = 16;
    private static final int HEIGHT = 16;
    private static final int TOTAL_LEDS = WIDTH * HEIGHT;

    // grid stores snake body age (0 = empty)
    private final int[] grid = new int[TOTAL_LEDS];

    // ---------------- GAME STATE ----------------

    private int snakeLength = 4;
    private int headPos = 252;
    private int foodPos = -1;

    private boolean dead = false;

    private Direction direction = Direction.LEFT;

    private enum Direction {
        UP, DOWN, LEFT, RIGHT
    }

    // ---------------- SPEED CONTROL ----------------

    private final Timer moveTimer = new Timer();
    private double moveDelay = 0.25; // seconds per move

    private final Random random = new Random();

    // ---------------- CONSTRUCTOR ----------------

    public LedSubsystem() {

        led = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(TOTAL_LEDS);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        initializeSnake();
        spawnFood();

        moveTimer.start();
    }

    // ---------------- INITIALIZE ----------------

    private void initializeSnake() {
        grid[255] = 1;
        grid[254] = 2;
        grid[253] = 3;
        grid[252] = 4;
        headPos = 252;
    }

    // ---------------- PERIODIC ----------------

    @Override
    public void periodic() {

        if (dead) {
            updateLEDs();
            return;
        }

        if (moveTimer.hasElapsed(moveDelay)) {
            moveSnake();
            moveTimer.reset();
            printBoard();
        }

        updateLEDs();
    }

    // ---------------- MOVEMENT ----------------

    private void moveSnake() {

        int row = headPos / WIDTH;
        int col = headPos % WIDTH;

        int newRow = row;
        int newCol = col;

        switch (direction) {
            case LEFT:  newCol--; break;
            case RIGHT: newCol++; break;
            case UP:    newRow--; break;
            case DOWN:  newRow++; break;
        }

        // Wall collision
        if (newRow < 0 || newRow >= HEIGHT || newCol < 0 || newCol >= WIDTH) {
            die();
            return;
        }

        int newHeadPos = newRow * WIDTH + newCol;

        // Self collision
        if (grid[newHeadPos] > 0) {
            die();
            return;
        }

        boolean ateFood = (newHeadPos == foodPos);

        if (ateFood) {
            snakeLength++;
            spawnFood();
        }

        // Age body segments
        for (int i = 0; i < TOTAL_LEDS; i++) {
            if (grid[i] > 0) {
                grid[i]++;
                if (grid[i] > snakeLength) {
                    grid[i] = 0;
                }
            }
        }

        // Place new head
        grid[newHeadPos] = 1;
        headPos = newHeadPos;
    }

    // ---------------- FOOD ----------------

    private void spawnFood() {
        int pos;
        do {
            pos = random.nextInt(TOTAL_LEDS);
        } while (grid[pos] > 0);

        foodPos = pos;
    }

    // ---------------- DEATH ----------------

    private void die() {
        System.out.println("===== GAME OVER =====");
        dead = true;
    }

    // ---------------- LED UPDATE ----------------

    private void updateLEDs() {

        for (int i = 0; i < buffer.getLength(); i++) {

            if (dead) {
                buffer.setRGB(i, 255, 0, 0); // Red screen when dead
            }
            else if (i == foodPos) {
                buffer.setRGB(i, 255, 0, 0); // Red food
            }
            else if (grid[i] > 0) {
                if (i == headPos) {
                    buffer.setRGB(i, 0, 255, 255); // Cyan head
                } else {
                    buffer.setRGB(i, 0, 255, 0); // Green body
                }
            }
            else {
                buffer.setRGB(i, 0, 0, 0); // Off
            }
        }

        led.setData(buffer);
    }

    // ---------------- BOARD PRINT ----------------

    private void printBoard() {

        System.out.println("=================================");

        for (int row = 0; row < HEIGHT; row++) {
            for (int col = 0; col < WIDTH; col++) {

                int index = row * WIDTH + col;

                if (index == headPos) {
                    System.out.print("H ");
                }
                else if (index == foodPos) {
                    System.out.print("F ");
                }
                else if (grid[index] > 0) {
                    System.out.print("O ");
                }
                else {
                    System.out.print(". ");
                }
            }
            System.out.println();
        }

        System.out.println("Length: " + snakeLength);
        System.out.println("Speed (sec per move): " + moveDelay);
    }

    // ---------------- SPEED CONTROL ----------------

    public void setSpeed(double secondsPerMove) {
        moveDelay = Math.max(0.05, secondsPerMove);
    }

    public void increaseSpeed() {
        moveDelay = Math.max(0.05, moveDelay - 0.05);
    }

    public void decreaseSpeed() {
        moveDelay += 0.05;
    }

    // ---------------- DIRECTION CONTROL ----------------

    public void setUp()    { if (direction != Direction.DOWN)  direction = Direction.UP; }
    public void setDown()  { if (direction != Direction.UP)    direction = Direction.DOWN; }
    public void setLeft()  { if (direction != Direction.RIGHT) direction = Direction.LEFT; }
    public void setRight() { if (direction != Direction.LEFT)  direction = Direction.RIGHT; }

    // ---------------- OPTIONAL RESET ----------------

    public void resetGame() {
        for (int i = 0; i < TOTAL_LEDS; i++) {
            grid[i] = 0;
        }

        snakeLength = 4;
        dead = false;
        direction = Direction.LEFT;

        initializeSnake();
        spawnFood();
    }
        */
}