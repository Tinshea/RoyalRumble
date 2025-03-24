package weUsedToLoveAsiats;

import weUsedToLoveAsiats.tools.CartCoordinate;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.stream.Collectors;

class IntCoord {
    public IntCoord parent;
    public int x;
    public int y;

    public IntCoord(int x, int y, IntCoord parent) {
        this.parent = parent;
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "IntCoord{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }
}

public class Astar {

    public static ArrayList<CartCoordinate> aStar(CartCoordinate start, CartCoordinate end, ArrayList<CartCoordinate> obstacle) {
        int[][] array = new int[31][21];
        for (int i = 0; i < array.length; i++) {
            for (int j = 0; j < array[i].length; j++) {
                final CartCoordinate coord = new CartCoordinate(i * 100, j * 100);
                long obs = obstacle.stream().filter(o -> o.distance(coord) < 100).count();
                double distance = end.distance(coord);
                array[i][j] = obs == 0 ? (int) Math.round(distance) : Integer.MAX_VALUE;
            }
        }

        IntCoord[][] coords = new IntCoord[31][21];

        for (int i = 0; i < coords.length; i++) {
            for (int j = 0; j < coords[i].length; j++) {
                coords[i][j] = new IntCoord(i, j, null);
            }
        }

        ArrayList<IntCoord> openList = new ArrayList<>();
        ArrayList<IntCoord> closedList = new ArrayList<>();

        openList.add(coords[(int) (Math.round(start.getX() / 100))][(int) (Math.round(start.getY() / 100))]);

        while (!openList.isEmpty()) {
            IntCoord current = openList.stream().min(Comparator.comparing(co -> array[co.x][co.y])).get();
            openList.remove(current);
            closedList.add(current);
            if (array[current.x][current.y] < 100) {
                ArrayList<IntCoord> backtrack = new ArrayList<>();
                while (current.parent != null) {
                    backtrack.add(0, current);
                    current = current.parent;
                }
                return new ArrayList<>(backtrack.stream()
                        .map(intCoord -> new CartCoordinate(intCoord.x * 100, intCoord.y * 100))
                        .collect(Collectors.toList()));
            }

            ArrayList<IntCoord> tmp = new ArrayList<>();
            if (current.x > 0) {
                tmp.add(coords[current.x - 1][current.y]);
                if (current.y > 0) {
                    tmp.add(coords[current.x - 1][current.y - 1]);
                }
                if (current.y < 19) {
                    tmp.add(coords[current.x - 1][current.y + 1]);
                }
            }
            if (current.x < 29) {
                tmp.add(coords[current.x + 1][current.y]);
                if (current.y > 0) {
                    tmp.add(coords[current.x + 1][current.y - 1]);
                }
                if (current.y < 19) {
                    tmp.add(coords[current.x + 1][current.y + 1]);
                }
            }
            if (current.y > 0) {
                tmp.add(coords[current.x][current.y - 1]);
            }
            if (current.y < 19) {
                tmp.add(coords[current.x][current.y + 1]);
            }

            for (IntCoord intCoord : tmp) {
                if (closedList.contains(intCoord)) continue;
                if (openList.contains(intCoord)) {
                    if (array[current.x][current.y] < array[intCoord.parent.x][intCoord.parent.y]) {
                        intCoord.parent = current;
                    }
                    continue;
                }
                intCoord.parent = current;
                openList.add(intCoord);
            }
        }

        return new ArrayList<>();
    }

    public static void main(String[] args) {
        CartCoordinate start = new CartCoordinate(200, 200);
        CartCoordinate end = new CartCoordinate(200, 1000);
        CartCoordinate obstacle = new CartCoordinate(200, 600);
        ArrayList<CartCoordinate> obstacles = new ArrayList<>();
        obstacles.add(obstacle);
        System.out.println(aStar(start, end, obstacles));
    }
}
