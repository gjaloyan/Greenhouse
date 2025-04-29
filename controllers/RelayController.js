import MQTTClient from '../service/mqtt-backend.js';

// ============ CONSTANTS ============

// Relay definitions
const relayConfig = {
  1: { name: 'Relay 1', description: 'Relay 1', id: 1 },
  2: { name: 'Relay 2', description: 'Relay 2', id: 2 },
  3: { name: 'Relay 3', description: 'Relay 3', id: 3 },
  4: { name: 'Relay 4', description: 'Relay 4', id: 4 },
  5: { name: 'Relay 5', description: 'Relay 5', id: 5 },
  6: { name: 'Relay 6', description: 'Relay 6', id: 6 },
  7: { name: 'Relay 7', description: 'Relay 7', id: 7 },
  '8': { name: 'Relay 8', description: 'Relay 8', id: 8 },
  9: { name: 'Relay 9', description: 'Relay 9', id: 9 },
  10: { name: 'Relay 10', description: 'Relay 10', id: 10 },
  11: { name: 'Relay 11', description: 'Relay 11', id: 11 },
  12: { name: 'Relay 12', description: 'Relay 12', id: 12 },
  13: { name: 'Relay 13', description: 'Relay 13', id: 13 },
  14: { name: 'Relay 14', description: 'Relay 14', id: 14 },
  15: { name: 'Relay 15', description: 'Relay 15', id: 15 },
  16: { name: 'Relay 16', description: 'Relay 16', id: 16 },
  17: { name: 'Relay 17', description: 'Relay 17', id: 17 },
  18: { name: 'Relay 18', description: 'Relay 18', id: 18 },
  19: { name: 'Relay 19', description: 'Relay 19', id: 19 },
  20: { name: 'Relay 20', description: 'Relay 20', id: 20 },
  21: { name: 'Relay 21', description: 'Relay 21', id: 21 },
  22: { name: 'Relay 22', description: 'Relay 22', id: 22 },
  23: { name: 'Relay 23', description: 'Relay 23', id: 23 },
  24: { name: 'Relay 24', description: 'Relay 24', id: 24 },
  25: { name: 'Relay 25', description: 'Relay 25', id: 25 },
  26: { name: 'Relay 26', description: 'Relay 26', id: 26 },
  27: { name: 'Relay 27', description: 'Relay 27', id: 27 },
  28: { name: 'Relay 28', description: 'Relay 28', id: 28 },
  29: { name: 'Relay 29', description: 'Relay 29', id: 29 },
  30: { name: 'Relay 30', description: 'Relay 30', id: 30 },
  31: { name: 'Relay 31', description: 'Relay 31', id: 31 },
  32: { name: 'Relay 32', description: 'Relay 32', id: 32 },
};

// MQTT Topics
const TOPICS = {
  COMMAND_ON: 'relay/command/on',
  COMMAND_OFF: 'relay/command/off',
  STATUS: 'relay/status',
  STATUS_REQUEST: 'relay/status/get',
};

const TIMEOUT = {
  STATE_VERIFY: 500, // Wait after sending command before verification
  ESP_RESPONSE: 2000, // Max wait for ESP8266 response
};

const UNKNOWN_STATE = 'unknown';

// ============ HELPERS ============

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

const now = () => Date.now();

const buildMessage = (payload) => (typeof payload === 'object' ? payload : { action: payload });

// Accept Buffer or string input, return parsed object or null
const safeJsonParse = (data) => {
  try {
    const str = typeof data === 'string' ? data : data.toString();
    return JSON.parse(str);
  } catch {
    return null;
  }
};

// ============ STATE ============

// { state: 'ON' | 'OFF' | 'unknown', lastUpdate: number }
const relayStates = new Map(
  Object.keys(relayConfig).map((id) => [id, { state: UNKNOWN_STATE, lastUpdate: 0 }])
);

const updateRelayState = (id, state) => {
  if (!relayConfig[id]) return false;
  relayStates.set(id, { state, lastUpdate: now() });
  return true;
};

const getRelayState = (id) => {
  const state = relayStates.get(String(id))?.state ?? UNKNOWN_STATE;
  return state;
};

const clearStaleStates = (ids) => {
  const threshold = 5000; // 5 s
  ids.forEach((id) => {
    const entry = relayStates.get(id);
    if (entry && now() - entry.lastUpdate > threshold) entry.state = UNKNOWN_STATE;
  });
};

// ============ MQTT ============

async function publishCommand(topic, payload) {
  const result = await MQTTClient.publishToTopic(topic, buildMessage(payload));
  if (!result?.success) throw new Error(result?.error || 'MQTT publish failed');
  return result;
}

async function checkConnection() {
  return MQTTClient.checkGreenhouse();
}

// Subscribe to STATUS topic once at start-up
console.log('Setting up relay status listener…');
MQTTClient.subscribeToTopic(TOPICS.STATUS)
  .then((r) => console.log(`Subscription ${r.success ? 'succeeded' : 'failed'}`))
  .catch((e) => console.error('Subscription error:', e.message));

MQTTClient.onMessage(TOPICS.STATUS, (_, raw) => {
  const parsed = safeJsonParse(raw);
  if (!parsed || Array.isArray(parsed)) return;

  const singleId = parsed.relay_id ?? parsed.relayId;
  if (singleId && parsed.state) {
    updateRelayState(String(singleId), String(parsed.state).toUpperCase());
    return;
  }

  // Case 2: bulk format { "16":"ON", "17":"OFF", greenhouse_id:"..." }
  Object.entries(parsed).forEach(([key, value]) => {
    if (key === 'greenhouse_id') return; // skip meta-field
    updateRelayState(String(key), String(value).toUpperCase());
  });
});

// ============ CORE LOGIC ============

async function requestRelayStatus(target) {
  try {
    const ids = target === 'all' ? Object.keys(relayConfig) : [target];

    // Mark stale values as unknown before requesting fresh data
    clearStaleStates(ids);

    const connection = await checkConnection();
    if (connection.success) {
      // Ask ESP for new status and wait for response
      await publishCommand(TOPICS.STATUS_REQUEST, target);
      await delay(TIMEOUT.ESP_RESPONSE);
    }

    if (target === 'all') {
      return Object.fromEntries(ids.map((id) => [id, getRelayState(id)]));
    }
    const targettoString = target.toString();
    return getRelayState(targettoString);

  } catch (err) {
    console.error('Status request error:', err.message);
    if (target === 'all') {
      return Object.fromEntries(
        Object.keys(relayConfig).map((id) => [id, UNKNOWN_STATE])
      );
    }
    return UNKNOWN_STATE;
  }
}

async function controlRelay(relayId, desiredState) {
  try {
    if (!relayConfig[relayId]) throw new Error(`Relay '${relayId}' not found`);

    // Ensure we start from a known state (helpful for logging/debugging)
    await requestRelayStatus(relayId);

    const topic = desiredState.toUpperCase() === 'ON' ? TOPICS.COMMAND_ON : TOPICS.COMMAND_OFF;
    await publishCommand(topic, Number(relayId));
    console.log(`Command sent to ${relayId}: ${desiredState}`);

    // Give ESP enough time to publish the new status
    await delay(TIMEOUT.ESP_RESPONSE);

    const actualState = await requestRelayStatus(relayId);
    const stateMatch = actualState.toUpperCase() === desiredState.toUpperCase();

    return {
      relayId,
      ...relayConfig[relayId],
      actualState: actualState,
      requestedState: desiredState,
      stateMatch,
      success: true,
      ...(stateMatch ? {} : { warning: `State mismatch: requested ${desiredState}, ESP reports ${actualState}` }),
    };
  } catch (err) {
    console.error('Relay control failed:', err.message);
    return {
      relayId,
      ...relayConfig[relayId],
      state: UNKNOWN_STATE,
      requestedState: desiredState,
      success: false,
      error: err.message,
    };
  }
}

// ============ EXPRESS CONTROLLERS ============

const getRelayStatus = async (req, res) => {
  const connection = await checkConnection();
  if (!connection.success) return res.status(503).json(connection);

  const states = await requestRelayStatus('all');
  return res.json({ states, source: 'Greenghouse', timestamp: now() });
};

const getSpecificRelayStatus = async (req, res) => {
  const relayId = req.body.relayId;
  if (!relayConfig[relayId]) {
    return res.status(404).json({ message: `Relay '${relayId}' not found`, success: false });
  }

  const connection = await checkConnection();
  if (!connection.success) return res.status(503).json(connection);

  const state = await requestRelayStatus(relayId);
  const lastUpdate = relayStates.get(relayId)?.lastUpdate || 0;

  return res.json({ relayId, ...relayConfig[relayId], state, lastUpdate, source: 'ESP8266' });
};

const RelayCommand = async (req, res) => {
  const { relayId, command } = req.body;
  const result = await controlRelay(relayId, command);
  return res.status(result.success ? 200 : 503).json(result);
};

const getRelayList = async (req, res) => {
  const connection = await checkConnection();
  if (!connection.success) return res.status(503).json(connection);

  // Fetch latest states but ignore errors (fallback to cached ones)
  const latestStates = await requestRelayStatus('all').catch(() => ({}));

  const relays = Object.entries(relayConfig).map(([id, cfg]) => {
    const state = getRelayState(id);
    console.log(`Relay ${id} state: ${state}`);
    return {
      relayId: id,
      ...cfg,
      state,
      lastUpdate: relayStates.get(id)?.lastUpdate || 0,
    };
  });

  return res.json({
    relays,
    count: relays.length,
    source: Object.keys(latestStates).length ? 'ESP8266' : 'cache',
  });
};

// ============ ERROR HANDLING ============

process.on('unhandledRejection', (reason) => {
  console.error('Unhandled Promise Rejection:', reason);
});

// ============ EXPORTS ============

export default {
  RelayCommand,
  getRelayStatus,
  getSpecificRelayStatus,
  getRelayList,
  checkConnection,
};       