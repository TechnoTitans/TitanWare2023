package frc.robot.commands.pathfinding;

import java.util.*;

public class AStar {
    private static <T> List<Node<T>> reconstruct(final Node<T> start, final Node<T> goal) {
        Node<T> prev = goal.getPrev();

        final List<Node<T>> reversedPath = new ArrayList<>(List.of(goal, prev));
        while (prev != start) {
            reversedPath.add((prev = prev.getPrev()));
        }

        Collections.reverse(reversedPath);
        return reversedPath;
    }

    public static <T> Optional<List<Node<T>>> compute(final Node<T> start, final Node<T> goal) {
        if (start.equals(goal)) {
            return Optional.empty();
        }

        final PriorityQueue<Node<T>> openSet = new PriorityQueue<>();
        final Set<Node<T>> closedSet = new HashSet<>();

        start.setG(0d);
        start.setF(goal);
        openSet.add(start);

        while (!openSet.isEmpty()) {
            final Node<T> current = openSet.poll();
            if (current.equals(goal)) {
                return Optional.of(reconstruct(start, goal));
            }

            closedSet.add(current);

            for (final Node<T> neighbor : current.getNeighbors()) {
                if (closedSet.contains(neighbor)) {
                    continue;
                }

                final double tentativeGScore = current.getG() + current.heuristicValue(neighbor);
                if (!openSet.contains(neighbor) || tentativeGScore < neighbor.getG()) {
                    neighbor.setPrev(current);
                    neighbor.setG(tentativeGScore);
                    neighbor.setF(tentativeGScore + neighbor.heuristicValue(goal));

                    openSet.add(neighbor);
                }
            }
        }

        return Optional.empty();
    }
}
