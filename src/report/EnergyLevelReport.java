/*
 * Copyright 2016 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package report;


import core.DTNHost;
import core.SimError;
import core.UpdateListener;

/**
 * Node energy level report. Reports the energy level of all 
 * (or only some, see {@link #REPORTED_NODES}) nodes every 
 * configurable-amount-of seconds (see {@link #GRANULARITY}).
 * Works only if all nodes use energy model; see 
 * {@link routing.util.EnergyModel}.
 */
public class EnergyLevelReport extends SnapshotReport 
	implements UpdateListener {

	@Override
	protected void writeSnapshot(DTNHost h) {
		if (!h.toString().startsWith("D")) {
            return;
        }
		Double remaining = (Double)h.getComBus().
				getProperty(routing.util.EnergyModel.ENERGY_VALUE_ID);
			if (remaining == null) {
				throw new SimError("Host " + h +
						" is not using energy model");
			}
			 
	     Double movement = (Double) h.getComBus()
	                .getProperty(routing.util.EnergyModel.MOVEMENT_ENERGY_ID);
	     if (movement == null) movement = 0.0;
	     
//			write(h.toString() + " " +  format(value) +" movement: "+format(movJ));
			write(String.format("%s remaining:%s movement: %s",  h,    format(remaining), format(movement)));

	}
}
