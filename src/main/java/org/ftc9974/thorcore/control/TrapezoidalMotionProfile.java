package org.ftc9974.thorcore.control;

import org.ftc9974.thorcore.util.MathUtilities;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Implements simple motion profiling.
 *
 * This class takes a list of (x, y) coordinates, called Nodes. these nodes are used to construct
 * the motion profile.
 */
public class TrapezoidalMotionProfile {

    public static class Node implements Comparable<Node> {
        private double x, y;

        public Node(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public int compareTo(Node o) {
            if (x < o.x) return -1;
            if (x > o.x) return 1;
            return 0;
        }
    }

    private LinkedList<Node> nodes;

    public TrapezoidalMotionProfile(final Node... nodes) {
        Arrays.sort(nodes);
        this.nodes = new LinkedList<>(Arrays.asList(nodes));
    }

    public double apply(double x) {
        Node start = null, end = null;
        for (int i = 0; i < nodes.size(); i++) {
            Node current = nodes.get(i);
            if (current.x < x) {
                start = current;
            } else if (current.x > x) {
                end = current;
                break;
            }
        }
        if (start == null) {
            return nodes.getFirst().y;
        }
        if (end == null) {
            return nodes.getLast().y;
        }
        return MathUtilities.map(x, start.x, end.x, start.y, end.y);
    }
}
