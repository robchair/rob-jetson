import { useState } from "react";
import useROS from "./hooks/useROS";
import ManualPanel from "./components/ManualPanel";
import VoicePanel from "./components/VoicePanel";
import "./App.css";

const STATUS_LABEL = {
  connecting: "Connecting to ROS...",
  connected:  "ROS Connected",
  error:      "ROS Error — retrying...",
};

export default function App() {
  const [mode, setMode] = useState("manual");
  const { status, publish, stop } = useROS();

  return (
    <div className="app">

      {/* ── Status bar ── */}
      <div className="status-bar">
        <div className="ros-status">
          <div className={`ros-dot ${status}`} />
          <span>{STATUS_LABEL[status]}</span>
        </div>
        <span className="app-title">ROB Control Panel</span>
        <span className="mode-label">
          {mode === "manual" ? "Manual Control" : "Voice Control"}
        </span>
      </div>

      {/* ── Mode tabs ── */}
      <div className="mode-tabs">
        <button
          className={`tab ${mode === "manual" ? "active" : ""}`}
          onClick={() => setMode("manual")}
        >
          Manual
        </button>
        <button
          className={`tab ${mode === "voice" ? "active" : ""}`}
          onClick={() => setMode("voice")}
        >
          Voice
        </button>
      </div>

      {/* ── Panel ── */}
      <div className="panel">
        {mode === "manual"
          ? <ManualPanel publish={publish} stop={stop} />
          : <VoicePanel />
        }
      </div>

    </div>
  );
}