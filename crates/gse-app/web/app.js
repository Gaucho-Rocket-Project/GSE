/* Ground Support Equipment operator console.
 * Plain ES2019, no build step. Talks to the local Rust service. */
(function () {
  "use strict";

  var $ = function (id) { return document.getElementById(id); };
  var token = document.querySelector('meta[name="gse-token"]').content;

  var ACTION_LABELS = {
    fill_on: "FILL ON",
    fill_off: "FILL OFF",
    srv_on: "SRV ON",
    srv_off: "SRV OFF",
    ign_pwr: "IGN PWR",
    ign_safe: "IGN SAFE",
    reset_controls: "RESET CONTROLS",
    reset_fill: "RESET FILL SEQ",
    clear_faults: "CLEAR FAULTS",
    estop: "EMERGENCY STOP"
  };
  var SHORTCUT_LABELS = {
    fill_toggle: "FILL ON / OFF",
    srv_toggle: "SRV ON / OFF",
    ign_toggle: "IGN PWR / SAFE",
    estop: "All safe (E-STOP)",
    reset_controls: "Reset controls to OFF/SAFE",
    reset_fill: "Reset fill sequence",
    clear_faults: "Clear latched faults"
  };

  var snapshot = null;
  var config = null;
  var seen = new Set();
  var seenOrder = [];
  var logFilter = "";
  var follow = true;
  var recording = null;
  var pendingIgn = null;
  var toastTimer = null;
  var lastBeep = 0;
  var audioCtx = null;
  var fillTick = null;
  var clockTick = null;
  var lastVehicleOnline = null;
  var lastFillComplete = null;

  /* ------------------------------------------------------------------ */
  /* helpers                                                            */
  /* ------------------------------------------------------------------ */

  function api(path, body) {
    var opts = { method: body === undefined ? "GET" : "POST", headers: {} };
    opts.headers["X-GSE-Token"] = token;
    if (body !== undefined) {
      opts.headers["Content-Type"] = "application/json";
      opts.body = JSON.stringify(body);
    }
    return fetch(path, opts).then(function (res) {
      return res.text().then(function (text) {
        var data = null;
        try { data = text ? JSON.parse(text) : {}; } catch (e) { data = { raw: text }; }
        if (!res.ok) {
          var msg = (data && data.error) || ("HTTP " + res.status);
          throw new Error(msg);
        }
        return data;
      });
    });
  }

  function toast(msg, kind, ms) {
    var el = $("toast");
    el.textContent = msg;
    el.className = "toast" + (kind ? " " + kind : "");
    el.hidden = false;
    if (toastTimer) { clearTimeout(toastTimer); }
    toastTimer = setTimeout(function () { el.hidden = true; }, ms || 3600);
  }

  function beep(freq, duration) {
    if (!config || !config.sound) { return; }
    var now = Date.now();
    if (now - lastBeep < 350) { return; }
    lastBeep = now;
    try {
      if (!audioCtx) {
        var Ctx = window.AudioContext || window.webkitAudioContext;
        if (!Ctx) { return; }
        audioCtx = new Ctx();
      }
      var osc = audioCtx.createOscillator();
      var gain = audioCtx.createGain();
      osc.type = "sine";
      osc.frequency.value = freq;
      gain.gain.value = 0.06;
      osc.connect(gain);
      gain.connect(audioCtx.destination);
      var t = audioCtx.currentTime;
      gain.gain.setValueAtTime(0.06, t);
      gain.gain.exponentialRampToValueAtTime(0.0001, t + duration);
      osc.start(t);
      osc.stop(t + duration);
    } catch (e) { /* audio is best-effort */ }
  }

  function fmtMs(ms) {
    if (ms === null || ms === undefined) { return "–"; }
    if (ms < 1000) { return ms + " ms"; }
    var s = ms / 1000;
    if (s < 10) { return s.toFixed(1) + " s"; }
    if (s < 60) { return Math.round(s) + " s"; }
    var m = Math.floor(s / 60);
    var rem = Math.round(s % 60);
    return m + "m " + (rem < 10 ? "0" + rem : rem) + "s";
  }

  function fmtUptime(ms) {
    var total = Math.max(0, Math.floor((ms || 0) / 1000));
    var h = Math.floor(total / 3600);
    var m = Math.floor((total % 3600) / 60);
    var s = total % 60;
    return pad(h) + ":" + pad(m) + ":" + pad(s);
  }

  function pad(n) { return n < 10 ? "0" + n : String(n); }

  function clockOf(epochMs) {
    var d = new Date(epochMs);
    return pad(d.getHours()) + ":" + pad(d.getMinutes()) + ":" + pad(d.getSeconds());
  }

  function fmtCountdown(ms) {
    if (ms === null || ms === undefined || ms < 0) { return "–"; }
    var total = Math.ceil(ms / 1000);
    var m = Math.floor(total / 60);
    var s = total % 60;
    return m + ":" + pad(s);
  }

  function versionText(fw) {
    return fw ? "v" + fw : "–";
  }

  function deviceText(id) {
    if (id === null || id === undefined) { return "–"; }
    return "0x" + Number(id).toString(16).toUpperCase().padStart(4, "0");
  }

  /* ------------------------------------------------------------------ */
  /* combo handling                                                     */
  /* ------------------------------------------------------------------ */

  var MODIFIERS = ["ctrl", "alt", "shift", "meta"];

  function normalizeCombo(text) {
    if (!text) { return ""; }
    var parts = String(text).split("+");
    var mods = [];
    var key = "";
    for (var i = 0; i < parts.length; i++) {
      var p = parts[i].trim();
      if (!p) { continue; }
      var lp = p.toLowerCase();
      if (lp === "ctrl" || lp === "control") { mods.push("ctrl"); }
      else if (lp === "alt" || lp === "option") { mods.push("alt"); }
      else if (lp === "shift") { mods.push("shift"); }
      else if (lp === "meta" || lp === "cmd" || lp === "command" || lp === "win" || lp === "super") { mods.push("meta"); }
      else { key = p; }
    }
    var ordered = MODIFIERS.filter(function (m) { return mods.indexOf(m) >= 0; });
    if (key.length === 1) { key = key.toUpperCase(); }
    if (ordered.length) { return ordered.join("+") + "+" + key; }
    return key;
  }

  var KEY_NAMES = {
    " ": "Space",
    "Spacebar": "Space",
    "Esc": "Escape",
    "Left": "ArrowLeft",
    "Right": "ArrowRight",
    "Up": "ArrowUp",
    "Down": "ArrowDown",
    "Del": "Delete"
  };

  function comboFromEvent(e) {
    var key = e.key;
    if (!key) { return ""; }
    if (key === "Control" || key === "Shift" || key === "Alt" || key === "Meta") { return ""; }
    if (KEY_NAMES[key]) { key = KEY_NAMES[key]; }
    var mods = [];
    if (e.ctrlKey) { mods.push("ctrl"); }
    if (e.altKey) { mods.push("alt"); }
    if (e.shiftKey) { mods.push("shift"); }
    if (e.metaKey) { mods.push("meta"); }
    if (key === "Escape" && mods.length === 0) { return "escape"; }
    var out = mods.length ? mods.join("+") + "+" : "";
    return out + (key.length === 1 ? key.toUpperCase() : key);
  }

  function displayCombo(combo) {
    var norm = normalizeCombo(combo);
    return norm.split("+").map(function (p) {
      if (p === "ctrl") { return "Ctrl"; }
      if (p === "alt") { return "Alt"; }
      if (p === "shift") { return "Shift"; }
      if (p === "meta") { return "Meta"; }
      return p;
    }).join("+");
  }

  function actionForCombo(combo) {
    if (!config || !config.shortcuts) { return null; }
    var norm = normalizeCombo(combo).toLowerCase();
    var keys = Object.keys(config.shortcuts);
    for (var i = 0; i < keys.length; i++) {
      if (normalizeCombo(config.shortcuts[keys[i]]).toLowerCase() === norm) { return keys[i]; }
    }
    return null;
  }

  /* ------------------------------------------------------------------ */
  /* commands                                                           */
  /* ------------------------------------------------------------------ */

  function sendCommand(action) {
    if (!snapshot) { return; }
    var gate = snapshot.actions[action];
    if (gate && !gate.enabled) {
      toast(gate.reason || "Command blocked", "warn");
      beep(220, 0.12);
      return;
    }
    api("/api/command", { action: action }).catch(function (e) {
      toast(e.message, "error");
    });
  }

  function requestIgn() {
    if (!snapshot) { return; }
    var ignGate = snapshot.actions && snapshot.actions.ign_pwr;
    if (ignGate && !ignGate.enabled) {
      toast(ignGate.reason || "IGN PWR blocked", "warn");
      beep(220, 0.12);
      return;
    }
    if (config && config.confirm_ign) {
      pendingIgn = "ign_pwr";
      $("confirm-reason").textContent = "SRV is ON and the fill sequence is complete.";
      $("confirm").hidden = false;
      $("confirm-ok").focus();
      return;
    }
    sendCommand("ign_pwr");
  }

  function toggle(toggleAction) {
    if (!snapshot) { return; }
    var a = snapshot.actions;
    if (toggleAction === "fill_toggle") {
      if (a.fill_on.enabled) { sendCommand("fill_on"); }
      else if (a.fill_off.enabled) { sendCommand("fill_off"); }
      else { toast(a.fill_on.reason || a.fill_off.reason || "FILL unavailable", "warn"); }
    } else if (toggleAction === "srv_toggle") {
      if (a.srv_on.enabled) { sendCommand("srv_on"); }
      else if (a.srv_off.enabled) { sendCommand("srv_off"); }
      else { toast(a.srv_on.reason || a.srv_off.reason || "SRV unavailable", "warn"); }
    } else if (toggleAction === "ign_toggle") {
      if (a.ign_pwr.enabled) { requestIgn(); }
      else if (a.ign_safe.enabled) { sendCommand("ign_safe"); }
      else { toast(a.ign_pwr.reason || a.ign_safe.reason || "IGN unavailable", "warn"); }
    } else {
      sendCommand(toggleAction);
    }
  }

  /* ------------------------------------------------------------------ */
  /* rendering                                                          */
  /* ------------------------------------------------------------------ */

  function pill(id, state, text) {
    var el = $(id);
    el.dataset.state = state;
    var value = el.querySelector(".pill-value");
    if (value) { value.textContent = text; }
  }

  function renderToggle(cfg) {
    var btn = $(cfg.btnId);
    if (!btn) { return; }
    var nextAction = cfg.active ? cfg.offAction : cfg.onAction;
    var gate = cfg.actions[nextAction] || { enabled: false, reason: "" };
    btn.disabled = !gate.enabled;
    btn.title = gate.enabled ? (ACTION_LABELS[nextAction] || nextAction) : (gate.reason || "");
    btn.classList.toggle("is-on", !!cfg.active && !cfg.danger);
    btn.classList.toggle("is-pwr", !!cfg.active && !!cfg.danger);

    var swOn = $(cfg.swOnId);
    var swOff = $(cfg.swOffId);
    if (swOn) { swOn.classList.toggle("active", !!cfg.active); }
    if (swOff) { swOff.classList.toggle("active", !cfg.active); }

    var desc = $(cfg.descId);
    if (desc) { desc.textContent = cfg.desc; }

    var why = $(cfg.whyId);
    if (why) { why.textContent = gate.enabled ? "" : (gate.reason || ""); }
  }

  function setRailVal(id, text, cls) {
    var el = $(id);
    if (!el) { return; }
    el.textContent = text;
    el.className = "rail-v" + (cls ? " " + cls : "");
  }

  function render() {
    if (!snapshot) { return; }
    var s = snapshot;
    if (s.config) { config = s.config; }
    document.body.dataset.rev = s.rev;
    document.body.dataset.pending = s.pending ? s.pending.label : "";

    var groundOnline = s.ground.status === "online";
    var vehicleOnline = s.vehicle.status === "online";
    var gState = groundOnline ? "online" : (s.ground.status === "error" ? "error" : "scanning");
    pill("pill-ground", gState, s.ground.status + (groundOnline && s.ground.port ? " · " + s.ground.port : ""));
    pill("pill-vehicle", vehicleOnline ? "online" : (s.vehicle.status === "pinging" ? "pinging" : "offline"),
      s.vehicle.status + (vehicleOnline && s.vehicle.rtt_ms !== null ? " · " + s.vehicle.rtt_ms + " ms" : ""));
    pill("pill-link", vehicleOnline ? "online" : "idle", s.vehicle.rtt_ms !== null ? s.vehicle.rtt_ms + " ms" : "–");
    pill("pill-mode", s.sim ? "sim" : "online", s.sim ? "SIMULATOR" : "LIVE");

    if (vehicleOnline && lastVehicleOnline === false) { beep(880, 0.12); }
    if (!vehicleOnline && lastVehicleOnline === true) { beep(300, 0.25); toast("Vehicle link lost", "warn"); }
    lastVehicleOnline = vehicleOnline;

    var actions = s.actions || {};
    var vs = s.vehicle.state || {};
    // LED text comes from the confirmed control machine, not from a guess:
    // it only changes when the vehicle acknowledges the transition.
    var ctl = s.controls || {};
    var fillOn = ctl.fill ? ctl.fill === "ON" : !!vs.fill;
    var srvOn = ctl.srv ? ctl.srv === "ON" : !!vs.srv;
    var ignPwr = ctl.ign ? ctl.ign === "PWR" : !!vs.ign;
    setLed("led-fill", fillOn ? "ON" : "OFF", fillOn ? "on" : "");
    setLed("led-srv", srvOn ? "ON" : "OFF", srvOn ? "on" : "");
    setLed("led-ign", ignPwr ? "PWR" : "SAFE", ignPwr ? "pwr" : "");

    var target = ctl.target;
    renderToggle({
      btnId: "btn-fill",
      swOnId: "sw-fill-on",
      swOffId: "sw-fill-off",
      descId: "desc-fill",
      whyId: "why-fill",
      active: fillOn,
      danger: false,
      onAction: "fill_on",
      offAction: "fill_off",
      actions: actions,
      desc: target && target.fill !== ctl.fill
        ? ("Awaiting ACK → " + target.fill)
        : (fillOn ? "Fill valve open · click → OFF" : "Fill valve servo · click → ON")
    });
    renderToggle({
      btnId: "btn-srv",
      swOnId: "sw-srv-on",
      swOffId: "sw-srv-off",
      descId: "desc-srv",
      whyId: "why-srv",
      active: srvOn,
      danger: false,
      onAction: "srv_on",
      offAction: "srv_off",
      actions: actions,
      desc: target && target.srv !== ctl.srv
        ? ("Awaiting ACK → " + target.srv)
        : (srvOn ? "Vehicle servos ON · click → OFF" : "Vehicle servos · click → ON")
    });
    renderToggle({
      btnId: "btn-ign",
      swOnId: "sw-ign-on",
      swOffId: "sw-ign-off",
      descId: "desc-ign",
      whyId: "why-ign",
      active: ignPwr,
      danger: true,
      onAction: "ign_pwr",
      offAction: "ign_safe",
      actions: actions,
      desc: target && target.ign !== ctl.ign
        ? ("Awaiting ACK → " + target.ign)
        : (ignPwr
            ? "IGNITER ENERGISED · click → SAFE"
            : (actions.ign_pwr && actions.ign_pwr.enabled
                ? "Igniter armed · click → PWR"
                : "Igniter power · locked"))
    });

    Object.keys(actions).forEach(function (key) {
      var el = document.querySelector('[data-action="' + key + '"]');
      if (!el) { return; }
      el.disabled = !actions[key].enabled;
      el.title = actions[key].enabled ? (ACTION_LABELS[key] || key) : actions[key].reason;
      var why = el.querySelector(".ctl-why");
      if (why) { why.textContent = actions[key].reason || ""; }
    });

    var machineText;
    if (!ctl.ready) {
      machineText = "INIT · confirming OFF/SAFE";
      if (ctl.fill || ctl.srv || ctl.ign) {
        machineText += " · FILL " + (ctl.fill || "OFF") + " · SRV " +
          (ctl.srv || "OFF") + " · IGN " + (ctl.ign || "SAFE");
      }
    } else {
      machineText = "FILL " + (ctl.fill || "OFF") + " · SRV " +
        (ctl.srv || "OFF") + " · IGN " + (ctl.ign || "SAFE");
    }
    if (ctl.target) {
      machineText += " → FILL " + ctl.target.fill + " · SRV " +
        ctl.target.srv + " · IGN " + ctl.target.ign;
    }
    $("machine-text").textContent = machineText;
    document.body.dataset.controls = ctl.ready ? "ready" : "init";

    $("tele-summary").textContent = s.vehicle.summary || "–";
    $("tele-fw").textContent = versionText(s.vehicle.fw);
    $("tele-dev").textContent = deviceText(s.vehicle.device_id);
    $("tele-proto").textContent = "v" + s.ground.protocol;
    $("vehicle-age").textContent = vehicleOnline && s.vehicle.age_ms ? "updated " + fmtMs(s.vehicle.age_ms) + " ago" : "";

    renderChips(s);
    renderStats(s);
    renderFill(s);
    renderRail(s, groundOnline, vehicleOnline, fillOn, srvOn, ignPwr);

    if (s.pending) {
      $("pending-text").textContent = s.pending.label + " · try " + s.pending.attempts + " · " + fmtMs(s.pending.age_ms);
    } else {
      $("pending-text").textContent = "No command in flight";
    }

    $("scan-note").textContent = s.scan_note || (groundOnline ? "" : (s.port_hint ? "Looking for " + s.port_hint : ""));
    $("foot-config").textContent = "fill " + (s.config ? s.config.fill_seconds : "?") + " s · " +
      (s.config ? s.config.port + " @" + s.config.baud : "") + (s.sim ? " · simulator" : "");
    $("foot-session").textContent = s.log_path ? "log: " + s.log_path : "log: in memory";
    updateShortcutBadges();
    updateKeyFooter();
  }

  function renderRail(s, groundOnline, vehicleOnline, fillOn, srvOn, ignPwr) {
    var up = $("sys-uptime");
    if (up) { up.textContent = fmtUptime(s.uptime_ms); }
    setRailVal("proc-app", "ONLINE", "ok");
    setRailVal("proc-sim", s.sim ? "ACTIVE" : "DISABLED", s.sim ? "ok" : "dim");
    setRailVal("proc-lora",
      vehicleOnline ? ("ONLINE " + (s.vehicle.rtt_ms !== null ? s.vehicle.rtt_ms + "ms" : "")).trim()
        : (groundOnline ? "PINGING" : "OFFLINE"),
      vehicleOnline ? "ok" : (groundOnline ? "warn" : "dim"));

    var ctl = s.controls || {};
    var fill = s.fill || {};
    setRailVal("gate-init", ctl.ready ? "VERIFIED" : "PENDING", ctl.ready ? "ok" : "warn");
    setRailVal("gate-interlock",
      fillOn ? "SRV LOCKED" : (srvOn ? "FILL LOCKED" : "CLEAR"),
      (fillOn || srvOn) ? "warn" : "ok");
    setRailVal("gate-fill",
      fill.active ? "RUNNING" : (fill.seq_aborted ? "ABORTED" : (fill.seq_complete ? "COMPLETE" : "UNRUN")),
      fill.seq_aborted ? "hot" : (fill.seq_complete || fill.active ? "ok" : "dim"));
    setRailVal("gate-ign",
      ignPwr ? "ENERGISED" : (s.actions && s.actions.ign_pwr && s.actions.ign_pwr.enabled ? "ARMED" : "LOCKED"),
      ignPwr ? "hot" : (s.actions && s.actions.ign_pwr && s.actions.ign_pwr.enabled ? "ok" : "dim"));
    setRailVal("rail-pending", s.pending ? s.pending.label : "IDLE", s.pending ? "warn" : "dim");
  }

  function setLed(id, text, cls) {
    var el = $(id);
    el.textContent = text;
    el.className = "ctl-state" + (cls ? " " + cls : "");
  }

  function renderChips(s) {
    var box = $("chips-flags");
    box.textContent = "";
    (s.vehicle.flags || []).forEach(function (flag) {
      var chip = document.createElement("span");
      chip.className = "chip";
      var name = String(flag);
      if (name.indexOf("IGN PWR") >= 0 || name.indexOf("ESTOP") >= 0 || name.indexOf("LINK LOSS") >= 0) {
        chip.className += " hot";
      } else if (name.indexOf("FILL") >= 0 || name.indexOf("SRV ON") >= 0) {
        chip.className += " good";
      } else if (name.indexOf("REJECT") >= 0 || name.indexOf("INHIB") >= 0) {
        chip.className += " warn";
      }
      chip.textContent = name;
      box.appendChild(chip);
    });
    if (!box.children.length) {
      var none = document.createElement("span");
      none.className = "chip";
      none.textContent = "no active flags";
      box.appendChild(none);
    }

    var faults = $("faults");
    faults.textContent = "";
    (s.vehicle.fault_names || []).forEach(function (fault) {
      var el = document.createElement("span");
      el.className = "fault";
      el.textContent = "FAULT · " + fault;
      faults.appendChild(el);
    });
  }

  function renderStats(s) {
    var st = s.stats || {};
    $("stat-rx").textContent = st.rx_frames || 0;
    $("stat-tx").textContent = st.tx_frames || 0;
    $("stat-crc").textContent = st.crc_errors || 0;
    $("stat-malformed").textContent = st.malformed || 0;
    $("stat-retries").textContent = st.retries || 0;
    $("stat-reconnects").textContent = st.reconnects || 0;
    $("stat-acks").textContent = st.acks_ok || 0;
    $("stat-rejected").textContent = st.acks_rejected || 0;
    $("stat-commands").textContent = st.commands_sent || 0;
  }

  function renderFill(s) {
    var fill = s.fill || {};
    var state = s.vehicle.state || {};
    var duration = (fill.duration_s || (config ? config.fill_seconds : 60)) * 1000;
    var remaining = fill.remaining_ms;
    if (fill.ends_at && (fill.active || state.fill)) {
      remaining = Math.max(0, fill.ends_at - Date.now());
    }
    var pct = 0;
    if (fill.active && duration > 0) {
      pct = Math.min(100, Math.max(0, 100 - (remaining / duration) * 100));
    } else if (fill.seq_complete) {
      pct = 100;
    }
    var bar = $("fill-fill");
    bar.style.width = pct.toFixed(1) + "%";
    bar.className = "fill-fill" + (fill.seq_aborted ? " warn" : (fill.seq_complete && !fill.active ? " done" : ""));

    var text = "idle";
    if (fill.auto_off_pending) { text = "auto-off sending"; }
    else if (fill.active) { text = fill.remaining_ms > 0 && fill.remaining_ms < 5000 ? "closing soon" : "filling"; }
    else if (fill.seq_aborted) { text = "sequence aborted"; }
    else if (fill.seq_complete) { text = "sequence complete"; }
    else if (state.fill) { text = "valve open"; }
    $("fill-status").textContent = text;
    $("fill-remaining").textContent = fill.active ? fmtCountdown(remaining) + " remaining" : "–";
    $("fill-duration").textContent = (duration / 1000) + " s configured";

    if (fill.seq_complete && !fill.active && lastFillComplete === false) {
      beep(760, 0.2);
      toast("Fill sequence complete – SRV and IGN are now available", "ok", 5000);
    }
    if (fill.seq_aborted && !fill.active && lastFillComplete !== null) { beep(340, 0.2); }
    lastFillComplete = fill.seq_complete;
  }

  function updateShortcutBadges() {
    if (!config || !config.shortcuts) { return; }
    var map = {
      fill_on: "fill_toggle",
      srv_on: "srv_toggle",
      ign_pwr: "ign_toggle",
      estop: "estop",
      reset_controls: "reset_controls",
      reset_fill: "reset_fill",
      clear_faults: "clear_faults"
    };
    Object.keys(map).forEach(function (actionId) {
      var el = document.querySelector('[data-key-for="' + actionId + '"]');
      if (el) { el.textContent = displayCombo(config.shortcuts[map[actionId]] || ""); }
    });
  }

  function updateKeyFooter() {
    if (!config || !config.shortcuts) { return; }
    var sc = config.shortcuts;
    $("foot-keys").textContent = "keys: " +
      displayCombo(sc.fill_toggle) + " fill · " +
      displayCombo(sc.srv_toggle) + " srv · " +
      displayCombo(sc.ign_toggle) + " ign · " +
      displayCombo(sc.estop) + " e-stop · " +
      displayCombo(sc.reset_controls) + " reset all · " +
      displayCombo(sc.reset_fill) + " reset fill · " +
      displayCombo(sc.clear_faults) + " faults";
  }

  /* ------------------------------------------------------------------ */
  /* log                                                                */
  /* ------------------------------------------------------------------ */

  function addLog(entry, prepend) {
    if (!entry || !entry.id) { return; }
    if (seen.has(entry.id)) { return; }
    seen.add(entry.id);
    seenOrder.push(entry.id);
    if (seenOrder.length > 2000) {
      seen.delete(seenOrder.shift());
    }
    var row = document.createElement("div");
    row.className = "log-row " + (entry.level || "info");
    row.dataset.source = entry.source || "";
    var t = document.createElement("span");
    t.className = "t";
    t.textContent = clockOf(entry.t);
    var lvl = document.createElement("span");
    lvl.className = "lvl";
    lvl.textContent = entry.level || "info";
    var src = document.createElement("span");
    src.className = "src";
    src.textContent = entry.source || "";
    var msg = document.createElement("span");
    msg.className = "msg";
    msg.textContent = entry.msg + (entry.code ? "  [" + entry.code + "]" : "");
    row.appendChild(t);
    row.appendChild(lvl);
    row.appendChild(src);
    row.appendChild(msg);
    if (logFilter && row.dataset.source !== logFilter) { row.hidden = true; }

    var log = $("log");
    if (prepend) {
      log.insertBefore(row, log.firstChild);
    } else {
      log.appendChild(row);
      while (log.children.length > 800) { log.removeChild(log.firstChild); }
      if (follow) { log.scrollTop = log.scrollHeight; }
    }

    if (!prepend && (entry.level === "warn" || entry.level === "error")) {
      beep(entry.level === "error" ? 240 : 380, 0.16);
    }
  }

  function loadLog() {
    api("/api/log?limit=250").then(function (data) {
      var entries = data.entries || [];
      for (var i = 0; i < entries.length; i++) { addLog(entries[i], false); }
      var log = $("log");
      log.scrollTop = log.scrollHeight;
    }).catch(function () { /* the SSE stream will fill it in */ });
  }

  /* ------------------------------------------------------------------ */
  /* settings                                                           */
  /* ------------------------------------------------------------------ */

  function openSettings() {
    if (snapshot && snapshot.config) { config = snapshot.config; }
    var cfg = config || {};
    $("cfg-fill").value = cfg.fill_seconds || 60;
    $("cfg-confirm").checked = !!cfg.confirm_ign;
    $("cfg-sound").checked = !!cfg.sound;
    var bauds = [9600, 19200, 38400, 57600, 115200, 230400];
    var baudSel = $("cfg-baud");
    baudSel.textContent = "";
    bauds.forEach(function (b) {
      var opt = document.createElement("option");
      opt.value = String(b);
      opt.textContent = b + " baud";
      if (b === cfg.baud) { opt.selected = true; }
      baudSel.appendChild(opt);
    });
    renderShortcutEditor();
    refreshPorts();
    $("settings").showModal();
  }

  function refreshPorts() {
    var sel = $("cfg-port");
    var current = (config && config.port) || "auto";
    sel.textContent = "";
    var auto = document.createElement("option");
    auto.value = "auto";
    auto.textContent = "Auto-detect";
    sel.appendChild(auto);
    if (current !== "auto") {
      var own = document.createElement("option");
      own.value = current;
      own.textContent = current;
      own.selected = true;
      sel.appendChild(own);
    }
    $("port-hint").textContent = "Scanning…";
    api("/api/ports").then(function (data) {
      var found = 0;
      (data.ports || []).forEach(function (p) {
        found++;
        var opt = document.createElement("option");
        opt.value = p.path;
        opt.textContent = p.path + (p.label && p.label !== p.path ? "  · " + p.label : "");
        if (p.path === current) { opt.selected = true; }
        sel.appendChild(opt);
      });
      $("port-hint").textContent = found
        ? found + " port" + (found === 1 ? "" : "s") + " detected"
        : "No USB serial port detected – holding the ground board reset button usually helps";
    }).catch(function (e) {
      $("port-hint").textContent = e.message;
    });
  }

  function renderShortcutEditor() {
    var list = $("shortcut-list");
    list.textContent = "";
    var sc = (config && config.shortcuts) || {};
    Object.keys(SHORTCUT_LABELS).forEach(function (action) {
      var row = document.createElement("div");
      row.className = "shortcut-row";
      var label = document.createElement("span");
      label.textContent = SHORTCUT_LABELS[action];
      var btn = document.createElement("button");
      btn.type = "button";
      btn.dataset.action = action;
      btn.textContent = displayCombo(sc[action] || "—");
      btn.addEventListener("click", function () { startRecording(action, btn); });
      row.appendChild(label);
      row.appendChild(btn);
      list.appendChild(row);
    });
  }

  function startRecording(action, btn) {
    if (recording) { recording.btn.classList.remove("recording"); }
    recording = { action: action, btn: btn };
    btn.classList.add("recording");
    btn.textContent = "press keys…";
    btn.focus();
  }

  function stopRecording() {
    if (!recording) { return; }
    recording.btn.classList.remove("recording");
    recording = null;
    renderShortcutEditor();
  }

  function commitShortcut(combo) {
    var rec = recording;
    if (!rec) { return; }
    var display = displayCombo(combo);
    api("/api/shortcuts", { action: rec.action, combo: display }).then(function () {
      rec.btn.classList.remove("recording");
      recording = null;
      toast("Bound " + SHORTCUT_LABELS[rec.action] + " to " + display, "ok");
    }).catch(function (e) {
      rec.btn.classList.remove("recording");
      rec.btn.classList.add("conflict");
      rec.btn.textContent = display;
      recording = null;
      toast(e.message, "error", 5000);
    });
  }

  /* ------------------------------------------------------------------ */
  /* wiring                                                             */
  /* ------------------------------------------------------------------ */

  function wire() {
    document.querySelectorAll(".control-grid button[data-toggle]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        var toggleAction = btn.dataset.toggle;
        if (!toggleAction || recording) { return; }
        toggle(toggleAction);
      });
    });

    document.querySelectorAll(".control-grid button[data-action], .secondary-row button[data-action]").forEach(function (btn) {
      btn.addEventListener("click", function () {
        var action = btn.dataset.action;
        if (!action || recording) { return; }
        if (action === "ign_pwr") { requestIgn(); }
        else { sendCommand(action); }
      });
    });

    $("btn-connect").addEventListener("click", function () { api("/api/connect", {}).catch(function (e) { toast(e.message, "error"); }); });
    $("btn-disconnect").addEventListener("click", function () { api("/api/disconnect", {}).catch(function (e) { toast(e.message, "error"); }); });
    $("btn-settings").addEventListener("click", openSettings);
    $("btn-settings-close").addEventListener("click", function () { stopRecording(); $("settings").close(); });
    $("settings").addEventListener("close", stopRecording);
    $("btn-refresh-ports").addEventListener("click", refreshPorts);

    $("settings-form").addEventListener("submit", function (ev) {
      ev.preventDefault();
      var payload = {
        fill_seconds: parseInt($("cfg-fill").value, 10),
        port: $("cfg-port").value,
        baud: parseInt($("cfg-baud").value, 10),
        confirm_ign: $("cfg-confirm").checked,
        sound: $("cfg-sound").checked
      };
      api("/api/config", payload).then(function () {
        toast("Settings applied", "ok");
        $("settings").close();
      }).catch(function (e) { toast(e.message, "error", 5000); });
    });

    $("btn-reset-shortcuts").addEventListener("click", function () {
      api("/api/shortcuts", { reset: true }).then(function () {
        toast("Default key bindings restored", "ok");
      }).catch(function (e) { toast(e.message, "error"); });
    });

    $("log-filter").addEventListener("change", function () {
      logFilter = $("log-filter").value;
      document.querySelectorAll("#log .log-row").forEach(function (row) {
        row.hidden = !!logFilter && row.dataset.source !== logFilter;
      });
    });
    $("log-follow").addEventListener("change", function () { follow = $("log-follow").checked; });
    $("btn-clear-log").addEventListener("click", function () {
      api("/api/log/clear", {}).then(function () {
        $("log").textContent = "";
        seen.clear();
        seenOrder.length = 0;
      }).catch(function (e) { toast(e.message, "error"); });
    });
    $("log-download").addEventListener("click", function (ev) {
      ev.preventDefault();
      window.location.href = "/api/log.csv?token=" + encodeURIComponent(token);
    });

    $("confirm-cancel").addEventListener("click", function () {
      pendingIgn = null;
      $("confirm").hidden = true;
    });
    $("confirm-ok").addEventListener("click", function () {
      $("confirm").hidden = true;
      if (pendingIgn) { sendCommand(pendingIgn); }
      pendingIgn = null;
    });
    $("confirm").addEventListener("click", function (ev) {
      if (ev.target === $("confirm")) {
        pendingIgn = null;
        $("confirm").hidden = true;
      }
    });

    document.addEventListener("keydown", onKeyDown);
  }

  function onKeyDown(e) {
    if (recording) {
      e.preventDefault();
      e.stopPropagation();
      if (e.key === "Escape") { stopRecording(); return; }
      var combo = comboFromEvent(e);
      if (!combo) { return; }
      commitShortcut(combo);
      return;
    }
    if (!$("confirm").hidden) {
      if (e.key === "Escape") {
        pendingIgn = null;
        $("confirm").hidden = true;
      } else if (e.key === "Enter") {
        $("confirm-ok").click();
      }
      return;
    }
    if ($("settings").open) { return; }
    var tag = (e.target && e.target.tagName) || "";
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") { return; }
    if (e.ctrlKey || e.metaKey || e.altKey) {
      var withMod = actionForCombo(comboFromEvent(e));
      if (!withMod) { return; }
    }
    var action = actionForCombo(comboFromEvent(e));
    if (!action) { return; }
    e.preventDefault();
    toggle(action);
  }

  function connectEvents() {
    var es = new EventSource("/api/events?token=" + encodeURIComponent(token));
    es.addEventListener("state", function (ev) {
      var next;
      try { next = JSON.parse(ev.data); } catch (err) { return; }
      snapshot = next;
      try {
        render();
      } catch (err) {
        /* Keep the stream alive and make the fault visible in the console. */
        if (window.console && window.console.error) { window.console.error("render failed:", err); }
      }
    });
    es.addEventListener("log", function (ev) {
      var entry;
      try { entry = JSON.parse(ev.data); } catch (err) { return; }
      addLog(entry, false);
    });
    es.onerror = function () {
      /* EventSource retries on its own; surface a persistent hint. */
      if (snapshot) { $("scan-note").textContent = "Reconnecting to the GSE service…"; }
    };
  }

  function startTickers() {
    var clockEl = $("term-clock");
    function tickClock() {
      if (clockEl) { clockEl.textContent = clockOf(Date.now()); }
    }
    tickClock();
    if (clockTick) { clearInterval(clockTick); }
    clockTick = setInterval(tickClock, 1000);

    if (fillTick) { clearInterval(fillTick); }
    fillTick = setInterval(function () {
      if (snapshot) { renderFill(snapshot); }
    }, 200);
  }

  window.addEventListener("DOMContentLoaded", function () {
    wire();
    loadLog();
    connectEvents();
    startTickers();
    api("/api/state").then(function (state) {
      if (!snapshot) { snapshot = state; render(); }
      if (!config) { config = state.config; }
      updateShortcutBadges();
      updateKeyFooter();
    }).catch(function () { /* SSE will deliver state */ });
    setTimeout(function () {
      if (!snapshot) { $("scan-note").textContent = "Waiting for the GSE service…"; }
    }, 1500);
  });
})();
