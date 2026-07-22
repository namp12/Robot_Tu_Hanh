import React, { useState, useEffect, useRef } from 'react';
import './App.css';

function App() {
  const [telemetry, setTelemetry] = useState(null);
  const [status, setStatus] = useState('Disconnected');
  const [wsUrl, setWsUrl] = useState(`ws://${window.location.hostname}:8080`);
  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);

  const connectWebSocket = (url) => {
    if (wsRef.current) {
      wsRef.current.close();
    }
    setStatus('Connecting...');
    console.log(`Connecting to WebSocket: ${url}`);
    
    const ws = new WebSocket(url);
    wsRef.current = ws;

    ws.onopen = () => {
      setStatus('Connected');
      console.log('WebSocket connected');
    };

    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === 'telemetry') {
          setTelemetry(message.data);
        }
      } catch (err) {
        console.error('Error parsing WebSocket data:', err);
      }
    };

    ws.onclose = () => {
      setStatus('Disconnected');
      console.log('WebSocket disconnected, reconnecting in 2s...');
      reconnectTimeoutRef.current = setTimeout(() => {
        connectWebSocket(url);
      }, 2000);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
      ws.close();
    };
  };

  useEffect(() => {
    connectWebSocket(wsUrl);
    return () => {
      if (wsRef.current) wsRef.current.close();
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
    };
  }, []);

  const handleBeep = () => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send('BEEP');
      console.log('Sent BEEP command');
    } else {
      alert('WebSocket is not connected!');
    }
  };

  const handleSetMode = (modeVal) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(`SET_MODE:${modeVal}`);
      console.log(`Sent SET_MODE:${modeVal} command`);
    } else {
      alert('WebSocket is not connected!');
    }
  };

  const getModeName = (mode) => {
    switch (mode) {
      case 0: return 'MANUAL';
      case 1: return 'AUTO';
      case 2: return 'ROS2';
      default: return 'UNKNOWN';
    }
  };

  const getAutoStateName = (state) => {
    const states = [
      'AUTO_IDLE',
      'AUTO_FORWARD',
      'AUTO_STOP',
      'AUTO_BACKWARD',
      'AUTO_SCAN',
      'AUTO_TURN_LEFT',
      'AUTO_TURN_RIGHT',
      'AUTO_RECOVER'
    ];
    return states[state] || `STATE_${state}`;
  };

  return (
    <div className="dashboard-container">
      <header className="dashboard-header">
        <div className="header-title">
          <span className="logo-icon">🤖</span>
          <h1>Robot Mecanum Dashboard</h1>
        </div>
        <div className="connection-panel">
          <input
            type="text"
            value={wsUrl}
            onChange={(e) => setWsUrl(e.target.value)}
            placeholder="ws://localhost:8080"
            className="ws-input"
          />
          <button onClick={() => connectWebSocket(wsUrl)} className="connect-btn">
            Connect
          </button>
          <span className={`status-badge ${status.toLowerCase()}`}>
            {status}
          </span>
        </div>
      </header>

      <main className="dashboard-grid">
        {/* Distance Sensors Card */}
        <section className="dashboard-card card-distance">
          <h2>Distance Sensors</h2>
          <div className="distance-meters">
            <div className="distance-meter">
              <div className="meter-header">
                <h3>Front Sensor</h3>
                <span className="meter-value">
                  {telemetry ? `${telemetry.front_distance.toFixed(1)} cm` : 'N/A'}
                </span>
              </div>
              <div className="meter-bar-container">
                <div 
                  className={`meter-bar front-bar ${telemetry && telemetry.front_distance < 50 ? 'warning' : ''}`}
                  style={{ width: telemetry ? `${Math.min(100, (telemetry.front_distance / 200) * 100)}%` : '0%' }}
                />
              </div>
            </div>
            
            <div className="distance-meter">
              <div className="meter-header">
                <h3>Rear Sensor</h3>
                <span className="meter-value">
                  {telemetry ? `${telemetry.rear_distance.toFixed(1)} cm` : 'N/A'}
                </span>
              </div>
              <div className="meter-bar-container">
                <div 
                  className={`meter-bar rear-bar ${telemetry && telemetry.rear_distance < 50 ? 'warning' : ''}`}
                  style={{ width: telemetry ? `${Math.min(100, (telemetry.rear_distance / 200) * 100)}%` : '0%' }}
                />
              </div>
            </div>
          </div>
        </section>

        {/* IMU Orientation Card */}
        <section className="dashboard-card card-imu">
          <h2>IMU Yaw/Pitch/Roll</h2>
          <div className="imu-grid">
            <div className="imu-value-box">
              <span className="imu-label">Yaw</span>
              <span className="imu-data yaw-text">{telemetry ? `${telemetry.yaw.toFixed(1)}°` : 'N/A'}</span>
            </div>
            <div className="imu-value-box">
              <span className="imu-label">Pitch</span>
              <span className="imu-data pitch-text">{telemetry ? `${telemetry.pitch.toFixed(1)}°` : 'N/A'}</span>
            </div>
            <div className="imu-value-box">
              <span className="imu-label">Roll</span>
              <span className="imu-data roll-text">{telemetry ? `${telemetry.roll.toFixed(1)}°` : 'N/A'}</span>
            </div>
          </div>
          <div className="imu-details">
            <p>Accel X: {telemetry ? telemetry.accel_x : '0.000'} m/s²</p>
            <p>Accel Y: {telemetry ? telemetry.accel_y : '0.000'} m/s²</p>
            <p>Accel Z: {telemetry ? telemetry.accel_z : '0.000'} m/s²</p>
          </div>
        </section>

        {/* Status & Diagnostics Card */}
        <section className="dashboard-card card-status">
          <h2>Status & Modes</h2>
          <div className="status-grid">
            <div className="status-row">
              <span className="status-label">Current Mode</span>
              <span className="status-value mode-badge">
                {telemetry ? getModeName(telemetry.current_mode) : 'N/A'}
              </span>
            </div>
            <div className="status-row">
              <span className="status-label">Auto State</span>
              <span className="status-value state-badge">
                {telemetry ? getAutoStateName(telemetry.auto_state) : 'N/A'}
              </span>
            </div>
            <div className="status-row">
              <span className="status-label">Uptime</span>
              <span className="status-value">
                {telemetry ? `${(telemetry.timestamp_ms / 1000).toFixed(1)}s` : 'N/A'}
              </span>
            </div>
          </div>
          <div className="motor-speeds">
            <h3>Wheel PWM Feedback</h3>
            <div className="motor-speeds-grid">
              <div>FL: {telemetry ? telemetry.motor_fl_speed : 0}</div>
              <div>FR: {telemetry ? telemetry.motor_fr_speed : 0}</div>
              <div>RL: {telemetry ? telemetry.motor_rl_speed : 0}</div>
              <div>RR: {telemetry ? telemetry.motor_rr_speed : 0}</div>
            </div>
          </div>
        </section>

        {/* Control Panel Card */}
        <section className="dashboard-card card-controls">
          <h2>Control Panel</h2>
          <div className="control-buttons">
            <button onClick={handleBeep} className="control-btn beep-btn">
              📯 Bòi còi (Trigger Beep)
            </button>
            <div className="mode-selection-group">
              <h3>Change Operating Mode</h3>
              <div className="mode-btn-row">
                <button onClick={() => handleSetMode(0)} className="mode-btn manual-btn">
                  MANUAL
                </button>
                <button onClick={() => handleSetMode(1)} className="mode-btn auto-btn">
                  AUTO
                </button>
                <button onClick={() => handleSetMode(2)} className="mode-btn ros2-btn">
                  ROS2
                </button>
              </div>
            </div>
          </div>
        </section>
      </main>
      <footer className="dashboard-footer">
        <p>Robot Mecanum Low-Level Controller Status Bridge via WebSocket</p>
      </footer>
    </div>
  );
}

export default App;
