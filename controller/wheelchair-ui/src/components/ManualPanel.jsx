import { useEffect, useRef } from "react";

const LINEAR  = 0.5;  // m/s  — matches teleop_twist_keyboard default
const ANGULAR = 1.0;  // rad/s — matches teleop_twist_keyboard default

// Matches teleop_twist_keyboard key bindings exactly
const KEY_MAP = {
  u: { linear:  LINEAR,  angular:  ANGULAR }, // forward-left
  i: { linear:  LINEAR,  angular:  0       }, // forward
  o: { linear:  LINEAR,  angular: -ANGULAR }, // forward-right
  j: { linear:  0,       angular:  ANGULAR }, // turn left
  k: { linear:  0,       angular:  0       }, // stop
  l: { linear:  0,       angular: -ANGULAR }, // turn right
  m: { linear: -LINEAR,  angular: -ANGULAR }, // back-left
  ",": { linear: -LINEAR, angular:  0      }, // backward
  ".": { linear: -LINEAR, angular:  ANGULAR }, // back-right
};

const BUTTONS = [
  [
    { key: "u", label: "↖" },
    { key: "i", label: "↑" },
    { key: "o", label: "↗" },
  ],
  [
    { key: "j", label: "←" },
    { key: "k", label: "■" },
    { key: "l", label: "→" },
  ],
  [
    { key: "m",  label: "↙" },
    { key: ",",  label: "↓" },
    { key: ".",  label: "↘" },
  ],
];

export default function ManualPanel({ publish, stop }) {
  const pressed = useRef(new Set());

  function sendVelocity() {
    // last key pressed wins
    const keys = [...pressed.current];
    if (keys.length === 0) { stop(); return; }
    const last = keys[keys.length - 1];
    const { linear, angular } = KEY_MAP[last];
    // k = stop
    if (last === "k") stop();
    else publish(linear, angular);
  }

  function btnDown(k) {
    if (k === "k") { stop(); return; }
    pressed.current.add(k);
    sendVelocity();
  }

  function btnUp(k) {
    pressed.current.delete(k);
    sendVelocity();
  }

  // keyboard support for desktop testing
  useEffect(() => {
    function onKeyDown(e) {
      const k = e.key.toLowerCase();
      if (!KEY_MAP[k] || pressed.current.has(k)) return;
      if (k === "k") { stop(); return; }
      pressed.current.add(k);
      sendVelocity();
    }
    function onKeyUp(e) {
      const k = e.key.toLowerCase();
      if (!KEY_MAP[k]) return;
      pressed.current.delete(k);
      sendVelocity();
    }
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, []);

  return (
    <div className="manual-panel">
      <div className="teleop-grid">
        {BUTTONS.map((row, ri) => (
          <div key={ri} className="teleop-row">
            {row.map(({ key, label }) => (
              <button
                key={key}
                className={`ctrl-btn ${key === "k" ? "stop-btn" : ""}`}
                onPointerDown={() => btnDown(key)}
                onPointerUp={() => btnUp(key)}
                onPointerLeave={() => btnUp(key)}
              >
                {label}
              </button>
            ))}
          </div>
        ))}
      </div>
      <p className="hint">Hold to move · ■ to stop · Release to stop</p>
    </div>
  );
}