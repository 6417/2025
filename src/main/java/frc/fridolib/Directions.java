package frc.fridolib;

public class Directions {
    // public enum RawDirection {
    // NONE(0, 0),
    // UP(1, 0),
    // RIGHT(0, 1),
    // DOWN(-1, 0),
    // LEFT(0, -1),

    // public Direction(int x, int y) {
    // }
    // }
	
	public static enum Direction1D {
		FORWARD(1),
		BACKWARD(-1);

		private final int direction;

		private Direction1D(int direction) {
			this.direction = direction;
		}

		public int getInt() {
			return direction;
		}
	}

    public static enum Pov {
        UP(0),
        UP_RIGHT(45),
        RIGHT(90),
        DOWN_RIGHT(135),
        DOWN(180),
        DOWN_LEFT(225),
        LEFT(270),
        UP_LEFT(315);

        private final int angle;

        private Pov(int angle) {
            this.angle = angle;
        }

        public int getDegrees() {
            return angle;
        }

        public double getRadians() {
            return Math.toRadians(angle);
        }
    }
}
