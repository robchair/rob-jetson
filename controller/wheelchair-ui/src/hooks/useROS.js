import { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";

const ROSBRIDGE_URL = "ws://localhost:9090";

export default function useROS() {
  const rosRef = useRef(null);
  const topicRef = useRef(null);
  const [status, setStatus] = useState("connecting");

  useEffect(() => {
    function connect() {
      const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
      rosRef.current = ros;

      ros.on("connection", () => {
        setStatus("connected");
        topicRef.current = new ROSLIB.Topic({
          ros,
          name: "/cmd_vel_keyboard",
          messageType: "geometry_msgs/Twist",
        });
      });

      ros.on("error", () => {
        setStatus("error");
        setTimeout(connect, 3000);
      });

      ros.on("close", () => {
        setStatus("connecting");
        setTimeout(connect, 3000);
      });
    }

    connect();
    return () => rosRef.current?.close();
  }, []);

  function publish(linear, angular) {
    topicRef.current?.publish({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    });
  }

  function stop() {
    publish(0, 0);
  }

  return { status, publish, stop };
}