package routing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import routing.util.EnergyModel;
import routing.util.RoutingInfo;

import core.Connection;
import core.Coord;
import core.DTNHost;
import core.Message;
import core.Settings;
import core.SimClock;

/**
 * TwoTierRouterAdvanced — strict two-tier DTN router with four borrowed
 * improvements from the ONE Simulator router library.
 *
 *   Node → Tower → Drone → Tower → Node
 * 
 */
public class TwoTierRouter extends ActiveRouter {

    /** Router namespace in default_settings.txt */
    public static final String ROUTER_NS = "TwoTierRouter";

    /**
     * How long (seconds) a node stays immune to a message it already
     * forwarded.  Prevents the same copy from being re-sent on every tick.
     * Borrowed from WaveRouter.IMMUNITY_S.
     * Default: 600 s (matches our msgTtl window).
     */
    public static final String IMMUNITY_TIME_S = "immunityTime";
    public static final double DEFAULT_IMMUNITY_TIME = 600.0;

    /**
     * Fraction of immunityTime during which a freshly-received message is
     * under "custody" and protected from buffer drops.
     * Borrowed from WaveRouter.CUSTODY_S.
     * Default: 0.5 → protected for 300 s after receipt.
     */
    public static final String CUSTODY_FRACTION_S = "custodyFraction";
    public static final double DEFAULT_CUSTODY_FRACTION = 0.5;

    private double immunityTime;
    private double custodyFraction; 

    /**
     * FEATURE 1 (WaveRouter): message ID → time this node last forwarded it.
     * While SimClock.getTime() < forwardTime + immunityTime, the node will
     * refuse to receive the same message again → kills flooding.
     */
    private Map<String, Double> immuneMessages;

    /**
     * FEATURE 2 (WaveRouter): message ID → time it was received here.
     * While SimClock.getTime() < receiveTime + immunityTime * custodyFraction,
     * this message is shielded from getNextMessageToRemove() → survives until
     * the next relay contact.
     */
    private Map<String, Double> custodyMessages;

    /**
     * FEATURE 3 (ProphetRouter): drone-host → encounter probability P(tower,drone).
     * Only populated and used on TOWER nodes. Lets a tower prefer the drone
     * it meets most often when multiple drones are in range.
     * Formula (standard PRoPHET):
     *   P_new = P_old + (1 - P_old) * P_INIT
     *   P ages every secondsInTimeUnit with factor GAMMA.
     */
    private Map<DTNHost, Double> dronePreds;
    private double lastAgeUpdate;

    private static final double P_INIT  = 0.75;
    private static final double GAMMA   = 0.98;
    private static final int    SECONDS_IN_UNIT = 30;

    private EnergyModel energyModel;
    private Coord lastLocation;
    
    public TwoTierRouter(Settings s) {
        super(s);
        Settings ns = new Settings(ROUTER_NS);
        immunityTime    = ns.contains(IMMUNITY_TIME_S)
                          ? ns.getDouble(IMMUNITY_TIME_S)
                          : DEFAULT_IMMUNITY_TIME;
        custodyFraction = ns.contains(CUSTODY_FRACTION_S)
                          ? ns.getDouble(CUSTODY_FRACTION_S)
                          : DEFAULT_CUSTODY_FRACTION;
         
        this.energyModel = s.contains(EnergyModel.INIT_ENERGY_S)
                ? new EnergyModel(s)
                : null;

    }

    protected TwoTierRouter(TwoTierRouter r) {
        super(r);
        this.immunityTime    = r.immunityTime;
        this.custodyFraction = r.custodyFraction; 
        this.immuneMessages  = new HashMap<>();
        this.custodyMessages = new HashMap<>();
        this.dronePreds      = new HashMap<>();
        this.lastAgeUpdate   = 0;
        
        this.energyModel = (r.energyModel != null) ? r.energyModel.replicate() : null;
        this.lastLocation = null;
    }

    /** Returns current energy. MAX_VALUE if no energy model (unlimited). */
    private double getCurrentEnergy() {
        if (energyModel == null) return Double.MAX_VALUE;
        return energyModel.getEnergy();
    }
 
    /** Returns initial energy for score calculation. */
    public double getInitialEnergy() {
        if (energyModel == null) return Double.MAX_VALUE;
        return energyModel.getInitialEnergy();
    }
  
    /**
     * FEATURE 3 (ProphetRouter): whenever this node (a Tower) connects to a
     * Drone, update the delivery probability for that drone so we can
     * prefer better-connected drones when forwarding.
     */
    @Override
    public void changedConnection(Connection con) {
        super.changedConnection(con);
        if (con.isUp() && isTower(getHost())) {
            DTNHost peer = con.getOtherNode(getHost());
            if (isDrone(peer)) {
                updatePredFor(peer);
            }
        }
    }

    private void updatePredFor(DTNHost drone) {
        double old = getPredFor(drone);
        dronePreds.put(drone, old + (1.0 - old) * P_INIT);
    }

    private double getPredFor(DTNHost drone) {
        ageDeliveryPreds();
        return dronePreds.getOrDefault(drone, 0.0);
    }

    private void ageDeliveryPreds() {
        double timeDiff = (SimClock.getTime() - lastAgeUpdate) / SECONDS_IN_UNIT;
        if (timeDiff == 0) return;
        double mult = Math.pow(GAMMA, timeDiff);
        for (Map.Entry<DTNHost, Double> e : dronePreds.entrySet()) {
            e.setValue(e.getValue() * mult);
        }
        lastAgeUpdate = SimClock.getTime();
    }

    /**
     * FEATURE 1 (WaveRouter): reject a message we recently forwarded.
     * This is the core anti-flooding gate. If we forwarded message M at
     * time T, we will refuse it again until T + immunityTime.
     */
//    @Override
//    protected int checkReceiving(Message m, DTNHost from) {
//        Double fwdTime = immuneMessages.get(m.getId());
//        if (fwdTime != null) {
//            if (fwdTime + immunityTime > SimClock.getTime()) {
//                return DENIED_POLICY; // still immune — reject re-flood
//            } else {
//                immuneMessages.remove(m.getId()); // immunity expired
//            }
//        }
//        return super.checkReceiving(m, from);
//    }
    @Override
    protected int checkReceiving(Message m, DTNHost from) {

        // topology enforcement
        if (!isAllowedRelay(from, getHost(), m)) {
            return DENIED_POLICY;
        }
         
        Double fwdTime = immuneMessages.get(m.getId());

        if (fwdTime != null) {
            if (fwdTime + immunityTime > SimClock.getTime()) {
                return DENIED_POLICY;
            } else {
                immuneMessages.remove(m.getId());
            }
        }

        return super.checkReceiving(m, from);
    }

    /**
     * FEATURE 2 (WaveRouter): when the buffer is full, skip messages that
     * are still under custody (recently received, waiting for next relay).
     * Falls back to the oldest unprotected message.
     */
    @Override
    protected Message getNextMessageToRemove(boolean excludeMsgBeingSent) {
        double now = SimClock.getTime();
        Message oldest = null;

        for (Message m : getMessageCollection()) {
            // Skip messages being actively sent
            if (excludeMsgBeingSent && isSending(m.getId())) continue;

            // FEATURE 2: skip messages still under custody
            Double custodyStart = custodyMessages.get(m.getId());
            if (custodyStart != null) {
                if (now < custodyStart + immunityTime * custodyFraction) {
                    continue; // protected — don't evict yet
                } else {
                    custodyMessages.remove(m.getId()); // custody expired
                }
            }

            if (oldest == null || m.getReceiveTime() < oldest.getReceiveTime()) {
                oldest = m;
            }
        }

        return oldest;
    }

    /**
     * FEATURE 4 (FirstContactRouter): when a NODE successfully sends its
     * message to a TOWER (the uplink hop), the node deletes its own copy.
     * This frees node buffer immediately so it can receive more messages
     * from the tower later (downlink).
     *
     * Towers and Drones keep their copy — they are relays and must hold
     * the message until the next hop confirms receipt.
     */
    @Override
    protected void transferDone(Connection con) {
        DTNHost me   = getHost();
        DTNHost peer = con.getOtherNode(me);
        Message m    = con.getMessage();

        if (m == null) return;

        // Node just uploaded to Tower → node no longer needs the copy
        if (isNode(me) && isTower(peer)) {
            deleteMessage(m.getId(), false);
        }
    }

    @Override
    public Message messageTransferred(String id, DTNHost from) {
        Message m = super.messageTransferred(id, from);
        // FEATURE 2: protect this freshly-received message from eviction
        custodyMessages.put(id, SimClock.getTime());
        // FEATURE 1: mark immunity so we don't re-accept the same message
        immuneMessages.put(id, SimClock.getTime());
        return m;
    }

    @Override
    public void update() {
        super.update();
        
        if (energyModel != null) { 
            // Step 1: register comBus properties on first tick
            energyModel.initComBus(getHost().getComBus());
 
            // Step 2: movement energy — drones only, only while moving
            if (isDrone(getHost())) {
                // clone() is critical: getLocation() returns a live reference.
                // Without clone, lastLocation always equals currentLocation
                // → isMoving always false → movement never charged.
                Coord currentLocation = getHost().getLocation().clone();
                if (lastLocation == null) {
                    lastLocation = currentLocation;
                } else {
                    boolean isMoving = !currentLocation.equals(lastLocation);
                    energyModel.updateMovementEnergy(SimClock.getTime(), isMoving);
                    lastLocation = currentLocation;
                }
            }
            if (energyModel.isOutOfEnergy()) {
                getHost().setFrozen(true);
                return;  
            }
        }

        if (isTransferring() || !canStartTransfer()) return;

        // Step 1: deliver directly to final destination if connected
        if (exchangeDeliverableMessages() != null) return;

        // Step 2: forward to next allowed relay hop
        tryForwardToAllowedPeers();
    }

    private void tryForwardToAllowedPeers() {
        List<Connection> connections = getConnections();
        if (connections.isEmpty()) return;

        // Sort messages oldest-first: prevents message starvation
        List<Message> messages = new ArrayList<>(getMessageCollection());
        Collections.sort(messages,
                (a, b) -> Double.compare(a.getReceiveTime(), b.getReceiveTime()));

        if (isTower(getHost())) {
            // FEATURE 3: towers pick the best drone first (highest P)
            tryForwardAsTower(messages, connections);
        } else {
            // Nodes and drones: strict topology, no preference ordering needed
            tryForwardStrict(messages, connections);
        }
    }

    /**
     * Tower forwarding — FEATURE 3 (ProphetRouter).
     * When multiple drones are in range, forward to the one with the
     * highest encounter probability first.
     * Also handles Tower → Node downlink (no preference needed there).
     */
//    private void tryForwardAsTower(List<Message> messages,
//                                    List<Connection> connections) {
//        // Split connections into drone-connections and node-connections
//        List<Connection> droneConns = new ArrayList<>();
//        List<Connection> nodeConns  = new ArrayList<>();
//
//        for (Connection con : connections) {
//            if (!con.isUp()) continue;
//            DTNHost peer = con.getOtherNode(getHost());
//            if (isDrone(peer))      droneConns.add(con);
//            else if (isNode(peer))  nodeConns.add(con);
//        }
//
//        // Sort drone connections by descending P value so highest-pred drone
//        // gets the message first
//        droneConns.sort((a, b) -> {
//            double pa = getPredFor(a.getOtherNode(getHost()));
//            double pb = getPredFor(b.getOtherNode(getHost()));
//            return Double.compare(pb, pa); // descending
//        });
//
//        // Try drone connections first (backhaul priority), then node connections
//        List<Connection> ordered = new ArrayList<>(droneConns);
//        ordered.addAll(nodeConns);
//
//        for (Connection con : ordered) {
//            DTNHost peer = con.getOtherNode(getHost());
////            for (Message m : messages) {
////                if (peer.getRouter().hasMessage(m.getId())) continue;
////                if (m.getTo() == getHost()) continue;
////                if (startTransfer(m, con) == RCV_OK) return;
////            }
//            for (Message m : messages) {
//                // already has message
//                if (peer.getRouter().hasMessage(m.getId())) {
//                    continue;
//                }
//                // don't send to self
//                if (m.getTo() == getHost()) {
//                    continue;
//                }
//                // Tower → Node ONLY if destination
//                if (isNode(peer) && m.getTo() != peer) {
//                    continue;
//                }
//                if (startTransfer(m, con) == RCV_OK) {
//                    return;
//                }
//            }
//        }
//    }
    private void tryForwardAsTower(List<Message> messages,
            List<Connection> connections) {
List<Connection> droneConns = new ArrayList<>();
List<Connection> nodeConns  = new ArrayList<>();

for (Connection con : connections) {
if (!con.isUp()) continue;
DTNHost peer = con.getOtherNode(getHost());
if (isDrone(peer))     droneConns.add(con);
else if (isNode(peer)) nodeConns.add(con);
}

// Case B: final delivery — check node connections first
for (Connection con : nodeConns) {
DTNHost peer = con.getOtherNode(getHost());
for (Message m : messages) {
if (m.getTo() != peer) continue;
if (peer.getRouter().hasMessage(m.getId())) continue;
if (startTransfer(m, con) == RCV_OK) return;
}
}

// Case A: relay — sort drones by score = P × energyFraction (descending)
droneConns.sort((a, b) -> {
double sa = getDroneScore(a.getOtherNode(getHost()));
double sb = getDroneScore(b.getOtherNode(getHost()));
return Double.compare(sb, sa);
});

for (Connection con : droneConns) {
DTNHost peer = con.getOtherNode(getHost());

for (Message m : messages) {
if (peer.getRouter().hasMessage(m.getId())) continue;
if (m.getTo() == getHost()) continue;
if (isDestinationDirectlyConnected(m)) continue;
if (startTransfer(m, con) == RCV_OK) return;
}
}
}
    /**
     * Combined score for drone relay selection.
     *
     *   score = P(tower, drone) × (currentEnergy / initialEnergy)
     *
     * Example:
     *   D0: P=0.87, energy=4000/5000=0.80 → score=0.696
     *   D1: P=0.92, energy= 400/5000=0.08 → score=0.074  ← avoided
     */
    private double getDroneScore(DTNHost drone) {
        double p = getPredFor(drone);
        TwoTierRouter dr = (TwoTierRouter) drone.getRouter();
        double energy    = dr.getCurrentEnergy();
        double maxEnergy = dr.getInitialEnergy();
        double fraction  = (maxEnergy > 0 && maxEnergy != Double.MAX_VALUE)
                           ? Math.min(1.0, energy / maxEnergy)
                           : 1.0;
        return p * fraction;
    }

    private boolean isDestinationDirectlyConnected(Message m) {
        for (Connection con : getConnections()) {
            if (con.isUp() && con.getOtherNode(getHost()) == m.getTo()) return true;
        }
        return false;
    }

    /**
     * Standard strict forwarding for nodes and drones.
     * Topology rules enforced; no probabilistic preference.
     */
    private void tryForwardStrict(List<Message> messages,
                                   List<Connection> connections) {
        for (Connection con : connections) {
            if (!con.isUp()) continue;
    
//            if (!isAllowedRelay(getHost(), peer, m)) continue;
//            for (Message m : messages) {
//                if (peer.getRouter().hasMessage(m.getId())) continue;
//                if (m.getTo() == getHost()) continue;
//                if (startTransfer(m, con) == RCV_OK) return;
//            }, 
            
            DTNHost peer = con.getOtherNode(getHost());
            for (Message m : messages) {

                if (!isAllowedRelay(getHost(), peer, m)) {
                    continue;
                }
                if (peer.getRouter().hasMessage(m.getId())) {
                    continue;
                }
                if (m.getTo() == getHost()) {
                    continue;
                }
                if (startTransfer(m, con) == RCV_OK) {
                    return;
                }
            }
        }
    }

    private boolean isAllowedRelay(DTNHost from, DTNHost to, Message m) {
        if (isNode(from))  return isTower(to);
        
//        if (isTower(from)) return isDrone(to) || isNode(to);
        if (isTower(from)) {
            if (isDrone(to)) {
                return true;
            }

            if (isNode(to) && m.getTo() == to) {
                return true;
            }

            return false;
        }
        if (isDrone(from)) return isTower(to);
        return false;
    }

    private String classifyHost(DTNHost h) {
        String id = h.toString();
        if (id.startsWith("T")) return "TOWER";
        if (id.startsWith("D")) return "DRONE";
        return "NODE";
    }

    private boolean isTower(DTNHost h) { return classifyHost(h).equals("TOWER"); }
    private boolean isDrone(DTNHost h)  { return classifyHost(h).equals("DRONE");  }
    private boolean isNode(DTNHost h)   { return classifyHost(h).equals("NODE");   }

    @Override
    public RoutingInfo getRoutingInfo() {
        ageDeliveryPreds();
        RoutingInfo top = super.getRoutingInfo();

        RoutingInfo immune = new RoutingInfo(
                "Immune to " + immuneMessages.size() + " messages");
        top.addMoreInfo(immune);

        RoutingInfo custody = new RoutingInfo(
                "Custody of " + custodyMessages.size() + " messages");
        top.addMoreInfo(custody);

        if (!dronePreds.isEmpty()) {
            RoutingInfo preds = new RoutingInfo(
                    dronePreds.size() + " drone delivery prediction(s)");
            for (Map.Entry<DTNHost, Double> e : dronePreds.entrySet()) {
                preds.addMoreInfo(new RoutingInfo(
                        String.format("%s : %.4f", e.getKey(), e.getValue())));
            }
            top.addMoreInfo(preds);
        }

        return top;
    }

    @Override
    public TwoTierRouter replicate() {
        return new TwoTierRouter(this);
    }
}