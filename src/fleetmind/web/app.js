// FleetMind AR operator console — talks to ROS2 over rosbridge (ws://:9090).
// Subscribes /fleet/states to draw the map + telemetry; publishes /fleet/command.

const WORLD = 12;                 // map spans -WORLD..WORLD meters
const FPV_HOST = `${location.hostname}:8080`;
const states = {};                // robot_id -> latest state
const trails = {};                // robot_id -> [{x,y}, ...]

const ros = new ROSLIB.Ros({ url: `ws://${location.hostname}:9090` });
const connEl = document.getElementById("conn");
ros.on("connection", () => { connEl.textContent = "● live"; connEl.className = "ok"; });
ros.on("error", () => { connEl.textContent = "● error"; connEl.className = "bad"; });
ros.on("close", () => { connEl.textContent = "● closed"; connEl.className = "bad"; });

const stateTopic = new ROSLIB.Topic({
  ros, name: "/fleet/states", messageType: "fleetmind_msgs/msg/RobotState",
});
const cmdTopic = new ROSLIB.Topic({
  ros, name: "/fleet/command", messageType: "fleetmind_msgs/msg/FleetCommand",
});

stateTopic.subscribe((msg) => {
  const yaw = 2 * Math.atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  const s = {
    id: msg.robot_id, type: msg.robot_type, task: msg.current_task,
    x: msg.pose.position.x, y: msg.pose.position.y, z: msg.pose.position.z,
    yaw, battery: msg.battery,
  };
  const isNew = !(msg.robot_id in states);
  states[msg.robot_id] = s;
  const tr = trails[msg.robot_id] || (trails[msg.robot_id] = []);
  const last = tr[tr.length - 1];
  if (last) {
    const d2 = (s.x - last.x) ** 2 + (s.y - last.y) ** 2;
    if (d2 < 0.01) { /* barely moved: skip */ }
    else { if (d2 > 1.0) tr.length = 0; tr.push({ x: s.x, y: s.y }); }
  } else {
    tr.push({ x: s.x, y: s.y });
  }
  if (tr.length > 200) tr.shift();
  if (isNew) refreshSelectors();
});

// ---- selectors ----
const sel = document.getElementById("sel");
const tgt = document.getElementById("tgt");
function refreshSelectors() {
  const ids = Object.keys(states).sort();
  const cur = sel.value, curT = tgt.value;
  sel.innerHTML = ["all", ...ids].map((i) => `<option>${i}</option>`).join("");
  tgt.innerHTML = ids.map((i) => `<option>${i}</option>`).join("");
  if (ids.includes(cur) || cur === "all") sel.value = cur;
  if (ids.includes(curT)) tgt.value = curT;
  updateFpv();
}
sel.addEventListener("change", updateFpv);

function updateFpv() {
  const r = sel.value === "all" ? Object.keys(states)[0] : sel.value;
  if (r) document.getElementById("fpv").src = `http://${FPV_HOST}/stream?robot=${r}&t=${Date.now()}`;
}

// ---- commands ----
function send(target, command, extra = {}) {
  cmdTopic.publish(new ROSLIB.Message({
    target, command,
    waypoints: extra.waypoints || [],
    follow_target: extra.follow_target || "",
    loop: extra.loop || false,
  }));
}
function cmd(command) {
  const target = sel.value;
  if (command === "FOLLOW") return send(target, "FOLLOW", { follow_target: tgt.value });
  if (command === "PATROL") {
    const wps = [{ x: 5, y: 5, z: 3 }, { x: -5, y: 5, z: 3 }, { x: -5, y: -5, z: 3 }, { x: 5, y: -5, z: 3 }];
    return send(target, "PATROL", { waypoints: wps, loop: true });
  }
  send(target, command);
}
function stopAll() { send("all", "STOP"); }
window.cmd = cmd; window.stopAll = stopAll;

// click map -> WAYPOINT
const map = document.getElementById("map");
map.addEventListener("click", (e) => {
  const rect = map.getBoundingClientRect();
  const px = (e.clientX - rect.left) / rect.width;
  const py = (e.clientY - rect.top) / rect.height;
  const wx = (px * 2 - 1) * WORLD;
  const wy = -(py * 2 - 1) * WORLD;
  const z = sel.value !== "all" && states[sel.value] && states[sel.value].type === "drone" ? 3 : 0;
  send(sel.value, "WAYPOINT", { waypoints: [{ x: wx, y: wy, z }] });
});

// ---- rendering ----
const ctx = map.getContext("2d");
function w2c(x, y) { return [(x / WORLD + 1) / 2 * 400, (1 - (y / WORLD + 1) / 2) * 400]; }

function draw() {
  ctx.fillStyle = "#0c120c"; ctx.fillRect(0, 0, 400, 400);
  ctx.strokeStyle = "#16221a"; ctx.lineWidth = 1;
  for (let i = 0; i <= 8; i++) {
    const p = i / 8 * 400;
    ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, 400); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(400, p); ctx.stroke();
  }
  ctx.strokeStyle = "#24402a";
  ctx.beginPath(); ctx.moveTo(200, 0); ctx.lineTo(200, 400); ctx.moveTo(0, 200); ctx.lineTo(400, 200); ctx.stroke();

  for (const id in states) {
    const s = states[id];
    const tr = trails[id] || [];
    ctx.strokeStyle = s.type === "drone" ? "rgba(80,200,255,.35)" : "rgba(120,255,120,.35)";
    ctx.beginPath();
    tr.forEach((p, i) => { const [cx, cy] = w2c(p.x, p.y); i ? ctx.lineTo(cx, cy) : ctx.moveTo(cx, cy); });
    ctx.stroke();

    const [cx, cy] = w2c(s.x, s.y);
    const col = s.type === "drone" ? "#50c8ff" : "#78ff78";
    ctx.fillStyle = col;
    ctx.beginPath(); ctx.arc(cx, cy, 6, 0, 7); ctx.fill();
    ctx.strokeStyle = col;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(-s.yaw) * 14, cy + Math.sin(-s.yaw) * 14); ctx.stroke();
    ctx.fillStyle = col; ctx.font = "11px monospace";
    ctx.fillText(`${id} ${s.task}`, cx + 9, cy - 7);
  }
  requestAnimationFrame(draw);
}
draw();

// telemetry table
setInterval(() => {
  const rows = Object.keys(states).sort().map((id) => {
    const s = states[id];
    return `<tr><td>${id}</td><td>${s.type}</td><td>${s.task}</td>
      <td>${s.x.toFixed(1)}</td><td>${s.y.toFixed(1)}</td><td>${s.z.toFixed(1)}</td>
      <td>${s.battery.toFixed(0)}%</td></tr>`;
  }).join("");
  document.querySelector("#fleet tbody").innerHTML = rows;
}, 300);
