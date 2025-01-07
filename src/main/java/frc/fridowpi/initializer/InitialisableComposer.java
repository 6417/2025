package frc.fridowpi.initializer;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.LinkedList;
import java.util.List;

public class InitialisableComposer {

    protected static class Node {
        public Initialisable initialisable;

        Node(Initialisable initialisable) {
            if (Initializer.getInstance().willBeInitialized(initialisable))
                Initializer.getInstance().removeInitialisable(initialisable);
            this.initialisable = initialisable;
        }
    }

    List<Node> queue = new LinkedList<>();

    protected InitialisableComposer(Initialisable initialisable) {
        queue.add(new Node(initialisable));
    }

    protected InitialisableComposer() {
    }

    protected static InitialisableComposer makeAfter(Initialisable after, Initialisable initialisable) {
        InitialisableComposer instance = new InitialisableComposer(initialisable);
        instance.queue.add(0, new Node(after));
        return instance;
    }

    private final Logger logger = LogManager.getLogger(InitialisableComposer.class);

    public InitialisableComposer then(Initialisable ini) {
        if (queue.stream().anyMatch((n) -> n.initialisable == ini)) {
            logger.warn("Initialisable was added twice with the 'then', the last call will be used");

            queue.removeIf((n) -> n.initialisable == ini);
        }
        queue.add(new Node(ini));
        return this;
    }

    protected List<Node> getQueue() {
        return queue;
    }

    public void close() {
        Initializer.getInstance().addComposer(this);
    }
}
