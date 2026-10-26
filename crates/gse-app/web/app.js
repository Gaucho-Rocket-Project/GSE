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
    srv_ign_on: "SRV+IGN ON",
    srv_ign_off: "SRV+IGN OFF",
    abort_countdown: "ABORT COUNTDOWN",
    reset_controls: "RESET CONTROLS",
    reset_fill: "RESET FILL SEQ",
    clear_faults: "CLEAR FAULTS",
    estop: "EMERGENCY STOP"
  };
  var SHORTCUT_LABELS = {
    fill_toggle: "FILL ON / OFF",
    srv_ign_toggle: "SRV+IGN ON / OFF (LIVE)",
    srv_toggle: "SRV ON / OFF (TEST)",
    ign_toggle: "IGN PWR / SAFE (TEST)",
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
  var lastCountdownSec = null;
  var audioCtx = null;
  var fillTick = null;
  var clockTick = null;
  var lastVehicleOnline = null;
  var lastFillComplete = null;
  var lastFillPaused = null;

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

  var USB_BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800];
  var UART_BAUDS = [2400, 4800, 9600, 19200, 38400, 57600, 115200];
  var RADIO_PART_NAMES = {
    "dx-lr42-24t12d": "DX-LR42-24T12D",
    "e28-2g4m27s": "E28-2G4M27S"
  };

  function radioIdOf(cfgOrLink) {
    var r = (cfgOrLink && cfgOrLink.radio) || "dx-lr42-24t12d";
    return String(r).toLowerCase() === "e28-2g4m27s" ? "e28-2g4m27s" : "dx-lr42-24t12d";
  }

  function radioPartOf(id) {
    return RADIO_PART_NAMES[id] || "DX-LR42-24T12D";
  }

  function isLiveMode() {
    var mode = (config && config.op_mode) || "live";
    return String(mode).toLowerCase() !== "test";
  }

  function countdownSecondsCfg() {
    return (config && config.countdown_seconds) ? config.countdown_seconds : 5;
  }

  function actionLabel(action) {
    if (action === "srv_ign_on") {
      return "SRV+IGN ON (" + countdownSecondsCfg() + " s COUNTDOWN)";
    }
    if (action === "ign_pwr") {
      return "IGN PWR (" + countdownSecondsCfg() + " s COUNTDOWN)";
    }
    return ACTION_LABELS[action] || action;
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

  function fmtLaunchSeconds(ms) {
    var clamped = Math.max(0, ms || 0);
    return (clamped / 1000).toFixed(1);
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
      $("confirm-reason").textContent = "SRV is ON and FILL is ready (" + countdownSecondsCfg() + " s countdown before IGN PWR).";
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
    } else if (toggleAction === "srv_ign_toggle") {
      if (a.srv_ign_on && a.srv_ign_on.enabled) { sendCommand("srv_ign_on"); }
      else if (a.srv_ign_off && a.srv_ign_off.enabled) { sendCommand("srv_ign_off"); }
      else {
        toast(
          (a.srv_ign_on && a.srv_ign_on.reason) ||
          (a.srv_ign_off && a.srv_ign_off.reason) ||
          "SRV+IGN unavailable",
          "warn"
        );
      }
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
    if (!el) { return; }
    el.dataset.state = state;
    var value = el.querySelector(".pill-value");
    if (value) { value.textContent = text; }
  }

  function renderToggle(cfg) {
    var btn = $(cfg.btnId);
    if (!btn) { return; }
    btn.hidden = !!cfg.hidden;
    var nextAction = (cfg.active || cfg.counting) ? cfg.offAction : cfg.onAction;
    var gate = cfg.actions[nextAction] || { enabled: false, reason: "" };
    btn.disabled = !gate.enabled;
    btn.title = gate.enabled
      ? (cfg.counting ? "ABORT COUNTDOWN" : actionLabel(nextAction))
      : (gate.reason || "");
    btn.classList.toggle("is-on", !!cfg.active && !cfg.danger);
    btn.classList.toggle("is-pwr", !!cfg.active && !!cfg.danger);
    btn.classList.toggle("is-counting", !!cfg.counting);

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

  function countdownRemainingMs(s) {
    var cd = (s && s.countdown) || {};
    if (!cd.active) { return 0; }
    if (cd.ends_at) {
      return Math.max(0, cd.ends_at - Date.now());
    }
    return Math.max(0, cd.remaining_ms || 0);
  }

  function renderCountdown(s) {
    var cd = (s && s.countdown) || {};
    var banner = $("countdown-banner");
    if (!banner) { return; }
    if (!cd.active) {
      banner.hidden = true;
      lastCountdownSec = null;
      return;
    }
    banner.hidden = false;
    var cdTarget = cd.target || (isLiveMode() ? "srv_ign" : "ign");
    var isSrvIgn = cdTarget === "srv_ign";
    var duration = cd.duration_ms || (countdownSecondsCfg() * 1000);
    var remaining = countdownRemainingMs(s);
    var wholeSec = Math.ceil(remaining / 1000);
    if (wholeSec > 0 && wholeSec !== lastCountdownSec) {
      lastCountdownSec = wholeSec;
      beep(660, 0.07);
    }
    var titleEl = $("countdown-title");
    if (titleEl) {
      titleEl.textContent = isSrvIgn
        ? ">> LIVE LAUNCH COUNTDOWN · SRV+IGN -> ON/PWR"
        : ">> IGNITION COUNTDOWN · IGN -> PWR";
    }
    var subEl = $("countdown-sub");
    if (subEl) {
      subEl.textContent = isSrvIgn
        ? "Outputs remain OFF/SAFE until T-0.0 s (FILL progress preserved on abort)"
        : "Aborting cancels IGN and returns SRV -> OFF while preserving FILL progress";
    }
    $("countdown-time").textContent = "T - " + fmtLaunchSeconds(remaining) + " s";
    var pct = duration > 0 ? Math.min(100, Math.max(0, (remaining / duration) * 100)) : 0;
    $("countdown-fill").style.width = pct.toFixed(1) + "%";

    if (isSrvIgn) {
      var ledLive = $("led-srv-ign");
      if (ledLive) {
        ledLive.textContent = "COUNTDOWN · " + fmtLaunchSeconds(remaining) + "s";
        ledLive.className = "ctl-state counting";
      }
      var descLive = $("desc-srv-ign");
      if (descLive) {
        descLive.textContent = "T - " + fmtLaunchSeconds(remaining) + " s · click → ABORT";
      }
      $("pending-text").textContent = "SRV+IGN countdown · T-" + fmtLaunchSeconds(remaining) + " s";
    } else {
      var ledTest = $("led-ign");
      if (ledTest) {
        ledTest.textContent = "COUNTDOWN · " + fmtLaunchSeconds(remaining) + "s";
        ledTest.className = "ctl-state counting";
      }
      var descTest = $("desc-ign");
      if (descTest) {
        descTest.textContent = "T - " + fmtLaunchSeconds(remaining) + " s · click → ABORT";
      }
      $("pending-text").textContent = "IGN countdown · T-" + fmtLaunchSeconds(remaining) + " s";
    }
  }

  function render() {
    if (!snapshot) { return; }
    var s = snapshot;
    if (s.config) { config = s.config; }
    var live = isLiveMode();
    document.body.dataset.rev = s.rev;
    document.body.dataset.mode = live ? "live" : "test";
    document.body.dataset.pending = s.pending ? s.pending.label : "";

    var groundOnline = s.ground.status === "online";
    var vehicleOnline = s.vehicle.status === "online";
    var gLink = (s.ground && s.ground.link) || {};
    var activeRadioId = groundOnline && gLink.radio ? radioIdOf(gLink) : radioIdOf(config);
    var activeRadioPart = radioPartOf(activeRadioId);
    var activeUsbBaud = (groundOnline && gLink.usb_baud) ? gLink.usb_baud : ((config && config.baud) || 115200);
    var activeUartBaud = (groundOnline && gLink.uart_baud) ? gLink.uart_baud : ((config && config.uart_baud) || 9600);
    var isE28 = activeRadioId === "e28-2g4m27s";

    var titleBrand = $("term-title");
    if (titleBrand) {
      titleBrand.textContent = "GROUND SUPPORT · NUCLEO-H753ZI · " + activeRadioPart;
    }

    var gState = groundOnline ? "online" : (s.ground.status === "error" ? "error" : "scanning");
    pill("pill-ground", gState, s.ground.status + (groundOnline && s.ground.port ? " · " + s.ground.port : ""));
    pill("pill-vehicle", vehicleOnline ? "online" : (s.vehicle.status === "pinging" ? "pinging" : "offline"),
      s.vehicle.status + (vehicleOnline && s.vehicle.rtt_ms !== null ? " · " + s.vehicle.rtt_ms + " ms" : ""));
    pill("pill-link", vehicleOnline ? "online" : "idle", s.vehicle.rtt_ms !== null ? s.vehicle.rtt_ms + " ms" : "–");
    pill(
      "pill-radio",
      s.link_reconfig ? "scanning" : (groundOnline ? "online" : "idle"),
      s.link_reconfig
        ? "switching…"
        : (isE28 ? "E28-2G4M27S · SPI1" : ("DX-LR42 · " + activeUartBaud))
    );
    pill("pill-opmode", live ? "live" : "test", live ? "LIVE" : "TEST");
    pill("pill-mode", s.sim ? "sim" : "online", s.sim ? "SIMULATOR" : "HARDWARE");

    $("h-controls").textContent = live ? "CONTROL CONSOLE // LIVE" : "CONTROL CONSOLE // TEST";
    var mk = $("machine-k");
    if (mk) { mk.textContent = live ? "CTRL STATE [LIVE]" : "CTRL STATE [TEST]"; }
    var grid = $("control-grid");
    if (grid) { grid.classList.toggle("is-live", live); }

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
    var srvIgnActive = srvOn || ignPwr;
    var cdActive = !!(s.countdown && s.countdown.active);
    var cdTarget = (s.countdown && s.countdown.target) || (live ? "srv_ign" : "ign");
    var fillPaused = !!(s.fill && s.fill.paused);

    setLed("led-fill", fillOn ? "ON" : (fillPaused ? "PAUSED" : "OFF"), fillOn ? "on" : (fillPaused ? "counting" : ""));
    setLed("led-srv", srvOn ? "ON" : "OFF", srvOn ? "on" : "");
    setLed("led-ign", ignPwr ? "PWR" : "SAFE", ignPwr ? "pwr" : "");
    setLed(
      "led-srv-ign",
      srvOn && ignPwr ? "ON/PWR" : (srvIgnActive ? (ctl.srv + "/" + ctl.ign) : "OFF/SAFE"),
      srvIgnActive ? "pwr" : ""
    );

    var target = ctl.target;
    renderToggle({
      btnId: "btn-fill",
      swOnId: "sw-fill-on",
      swOffId: "sw-fill-off",
      descId: "desc-fill",
      whyId: "why-fill",
      hidden: false,
      active: fillOn,
      danger: false,
      counting: false,
      onAction: "fill_on",
      offAction: "fill_off",
      actions: actions,
      desc: target && target.fill !== ctl.fill
        ? ("Awaiting ACK → " + target.fill)
        : (fillOn
            ? "Fill valve open · click → PAUSE/OFF"
            : (fillPaused
                ? ("Paused (" + fmtLaunchSeconds(s.fill.remaining_ms) + " s left) · click → RESUME")
                : "Fill valve servo · click → ON"))
    });

    renderToggle({
      btnId: "btn-srv-ign",
      swOnId: "sw-srv-ign-on",
      swOffId: "sw-srv-ign-off",
      descId: "desc-srv-ign",
      whyId: "why-srv-ign",
      hidden: !live,
      active: srvIgnActive,
      danger: true,
      counting: cdActive && cdTarget === "srv_ign",
      onAction: "srv_ign_on",
      offAction: "srv_ign_off",
      actions: actions,
      desc: target && (target.srv !== ctl.srv || target.ign !== ctl.ign)
        ? ("Awaiting ACK → " + target.srv + "/" + target.ign)
        : (srvIgnActive
            ? "ENERGISED (ON/PWR) · click → OFF"
            : (actions.srv_ign_on && actions.srv_ign_on.enabled
                ? ("Ready · press for " + countdownSecondsCfg() + " s countdown → ON")
                : "Combined launch channel · locked"))
    });

    renderToggle({
      btnId: "btn-srv",
      swOnId: "sw-srv-on",
      swOffId: "sw-srv-off",
      descId: "desc-srv",
      whyId: "why-srv",
      hidden: live,
      active: srvOn,
      danger: false,
      counting: false,
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
      hidden: live,
      active: ignPwr,
      danger: true,
      counting: cdActive && cdTarget === "ign",
      onAction: "ign_pwr",
      offAction: "ign_safe",
      actions: actions,
      desc: target && target.ign !== ctl.ign
        ? ("Awaiting ACK → " + target.ign)
        : (ignPwr
            ? "IGNITER ENERGISED · click → SAFE"
            : (actions.ign_pwr && actions.ign_pwr.enabled
                ? ("Igniter armed · press for " + countdownSecondsCfg() + " s countdown → PWR")
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

    if (s.pending) {
      $("pending-text").textContent = s.pending.label + " · try " + s.pending.attempts + " · " + fmtMs(s.pending.age_ms);
    } else if (!cdActive) {
      $("pending-text").textContent = "No command in flight";
    }
    renderCountdown(s);
    renderRail(s, live, groundOnline, vehicleOnline, fillOn, srvOn, ignPwr, cdActive);

    $("scan-note").textContent = s.link_reconfig
      ? ("Reconfiguring link hardware: " + s.link_reconfig)
      : (s.scan_note || (groundOnline ? "" : (s.port_hint ? "Looking for " + s.port_hint : "")));
    $("foot-config").textContent = "mode " + (live ? "LIVE" : "TEST") + " · fill " +
      (s.config ? s.config.fill_seconds : "?") + " s · countdown " + countdownSecondsCfg() + " s · " +
      activeRadioPart + " (" + (isE28 ? "SPI1 4MHz" : ("UART " + activeUartBaud)) + ") · " +
      (s.config ? s.config.port + " @" + activeUsbBaud : "") + (s.sim ? " · simulator" : "");
    $("foot-session").textContent = s.log_path ? "log: " + s.log_path : "log: in memory";
    updateShortcutBadges();
    updateKeyFooter();
  }

  function renderRail(s, live, groundOnline, vehicleOnline, fillOn, srvOn, ignPwr, cdActive) {
    var up = $("sys-uptime");
    if (up) { up.textContent = fmtUptime(s.uptime_ms); }
    setRailVal("proc-opmode", live ? "LIVE" : "TEST", live ? "hot" : "cyan");
    setRailVal("proc-app", "ONLINE", "ok");
    setRailVal("proc-sim", s.sim ? "ACTIVE" : "DISABLED", s.sim ? "ok" : "dim");
    var gLink = (s.ground && s.ground.link) || {};
    var activeRadioId = groundOnline && gLink.radio ? radioIdOf(gLink) : radioIdOf(config);
    var isE28 = activeRadioId === "e28-2g4m27s";
    var activeUsbBaud = (groundOnline && gLink.usb_baud) ? gLink.usb_baud : ((config && config.baud) || 115200);
    var activeUartBaud = (groundOnline && gLink.uart_baud) ? gLink.uart_baud : ((config && config.uart_baud) || 9600);
    var procLoraName = $("proc-lora-name");
    if (procLoraName) { procLoraName.textContent = isE28 ? "e28-spi1-rf" : "dx-lr42-rf"; }
    setRailVal("proc-lora",
      vehicleOnline ? ("ONLINE " + (s.vehicle.rtt_ms !== null ? s.vehicle.rtt_ms + "ms" : "")).trim()
        : (groundOnline ? "PINGING" : "OFFLINE"),
      vehicleOnline ? "ok" : (groundOnline ? "warn" : "dim"));
    setRailVal("rail-usb", activeUsbBaud + " Bd", "cyan");
    setRailVal("rail-uart", isE28 ? "SPI1 4MHz" : ("UART " + activeUartBaud), isE28 ? "ok" : "cyan");
    setRailVal("rail-flash", gLink.flash_saved ? "SAVED" : "DEFAULT", gLink.flash_saved ? "ok" : "dim");

    var ctl = s.controls || {};
    var fill = s.fill || {};
    setRailVal("gate-init", ctl.ready ? "VERIFIED" : "PENDING", ctl.ready ? "ok" : "warn");
    setRailVal("gate-interlock",
      fillOn ? "SRV LOCKED" : (srvOn ? "FILL LOCKED" : "CLEAR"),
      (fillOn || srvOn) ? "warn" : "ok");
    setRailVal("gate-fill",
      fill.active
        ? "RUNNING"
        : (fill.seq_aborted
            ? "ABORTED"
            : (fill.seq_complete ? "COMPLETE" : (fill.paused ? "PAUSED" : "UNRUN"))),
      fill.seq_aborted
        ? "hot"
        : (fill.seq_complete || fill.active || fill.paused ? "ok" : "dim"));

    var ignName = $("gate-ign-name");
    if (ignName) { ignName.textContent = live ? "srv+ign" : "ign gate"; }
    if (live) {
      setRailVal(
        "gate-ign",
        cdActive
          ? ("T-" + fmtLaunchSeconds(countdownRemainingMs(s)) + "s")
          : ((srvOn || ignPwr)
              ? "ON/PWR"
              : (s.actions && s.actions.srv_ign_on && s.actions.srv_ign_on.enabled ? "READY" : "LOCKED")),
        cdActive
          ? "warn"
          : ((srvOn || ignPwr)
              ? "hot"
              : (s.actions && s.actions.srv_ign_on && s.actions.srv_ign_on.enabled ? "ok" : "dim"))
      );
    } else {
      setRailVal("gate-ign",
        cdActive
          ? ("T-" + fmtLaunchSeconds(countdownRemainingMs(s)) + "s")
          : (ignPwr
              ? "ENERGISED"
              : (s.actions && s.actions.ign_pwr && s.actions.ign_pwr.enabled ? "ARMED" : "LOCKED")),
        cdActive
          ? "warn"
          : (ignPwr
              ? "hot"
              : (s.actions && s.actions.ign_pwr && s.actions.ign_pwr.enabled ? "ok" : "dim")));
    }
    setRailVal(
      "rail-pending",
      s.pending ? s.pending.label : (cdActive ? "COUNTDOWN" : "IDLE"),
      (s.pending || cdActive) ? "warn" : "dim"
    );
  }

  function setLed(id, text, cls) {
    var el = $(id);
    if (!el) { return; }
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
      if (name.indexOf("IGN PWR") >= 0 || name.indexOf("E-STOP") >= 0 || name.indexOf("ESTOP") >= 0 || name.indexOf("LINK SAFE") >= 0) {
        chip.className += " hot";
      } else if (name.indexOf("FILL PAUSED") >= 0 || name.indexOf("FILL ABORTED") >= 0 || name.indexOf("REJECT") >= 0 || name.indexOf("INHIB") >= 0) {
        chip.className += " warn";
      } else if (name.indexOf("FILL") >= 0 || name.indexOf("SRV ON") >= 0) {
        chip.className += " good";
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
    if ((fill.active || fill.paused) && duration > 0) {
      pct = Math.min(100, Math.max(0, 100 - (remaining / duration) * 100));
    } else if (fill.seq_complete) {
      pct = 100;
    }
    var bar = $("fill-fill");
    bar.style.width = pct.toFixed(1) + "%";
    bar.className = "fill-fill" + (
      fill.seq_aborted
        ? " warn"
        : (fill.paused ? " paused" : (fill.seq_complete && !fill.active ? " done" : ""))
    );

    var text = "idle";
    if (fill.auto_off_pending) { text = "auto-off sending"; }
    else if (fill.active) { text = fill.remaining_ms > 0 && fill.remaining_ms < 5000 ? "closing soon" : "filling"; }
    else if (fill.seq_aborted) { text = "sequence aborted"; }
    else if (fill.paused) { text = "paused · channels unlocked"; }
    else if (fill.seq_complete) { text = "sequence complete"; }
    else if (state.fill) { text = "valve open"; }
    $("fill-status").textContent = text;
    $("fill-remaining").textContent = fill.active
      ? fmtCountdown(remaining) + " remaining"
      : (fill.paused ? (fmtCountdown(remaining) + " remaining · click FILL to resume") : "–");
    $("fill-duration").textContent = (duration / 1000) + " s configured";

    if (fill.paused && !fill.active && lastFillPaused === false) {
      beep(580, 0.14);
      toast(
        isLiveMode()
          ? "Fill paused (" + fmtLaunchSeconds(remaining) + " s remaining) – SRV+IGN unlocked; click FILL to resume"
          : "Fill paused (" + fmtLaunchSeconds(remaining) + " s remaining) – SRV unlocked; click FILL to resume",
        "ok",
        5000
      );
    }
    if (fill.seq_complete && !fill.active && lastFillComplete === false) {
      beep(760, 0.2);
      toast(
        isLiveMode()
          ? "Fill sequence complete – SRV+IGN is now unlocked"
          : "Fill sequence complete – SRV and IGN are now available",
        "ok",
        5000
      );
    }
    if (fill.seq_aborted && !fill.active && lastFillComplete !== null) { beep(340, 0.2); }
    lastFillComplete = !!fill.seq_complete;
    lastFillPaused = !!fill.paused;
  }

  function updateShortcutBadges() {
    if (!config || !config.shortcuts) { return; }
    var map = {
      fill_on: "fill_toggle",
      srv_ign_on: "srv_ign_toggle",
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
    if (isLiveMode()) {
      $("foot-keys").textContent = "keys (LIVE): " +
        displayCombo(sc.fill_toggle) + " fill · " +
        displayCombo(sc.srv_ign_toggle) + " srv+ign · " +
        displayCombo(sc.estop) + " e-stop · " +
        displayCombo(sc.reset_controls) + " reset all · " +
        displayCombo(sc.reset_fill) + " reset fill · " +
        displayCombo(sc.clear_faults) + " faults";
    } else {
      $("foot-keys").textContent = "keys (TEST): " +
        displayCombo(sc.fill_toggle) + " fill · " +
        displayCombo(sc.srv_toggle) + " srv · " +
        displayCombo(sc.ign_toggle) + " ign · " +
        displayCombo(sc.estop) + " e-stop · " +
        displayCombo(sc.reset_controls) + " reset all · " +
        displayCombo(sc.reset_fill) + " reset fill · " +
        displayCombo(sc.clear_faults) + " faults";
    }
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

  function updateModeHint() {
    var hint = $("mode-hint");
    var sel = $("cfg-mode");
    var cdInput = $("cfg-countdown");
    var cdVal = (cdInput && parseInt(cdInput.value, 10)) || countdownSecondsCfg();
    if (!hint || !sel) { return; }
    if (sel.value === "test") {
      hint.textContent = "TEST exposes independent FILL, SRV, and IGN controls (SRV unlocks after FILL pauses or completes; IGN runs a " + cdVal + " s abortable countdown).";
    } else {
      hint.textContent = "LIVE combines SRV+IGN into one button unlocked after FILL pauses or completes, with a " + cdVal + " s abortable launch countdown.";
    }
  }

  function updateRadioHint() {
    var hint = $("radio-hint");
    var radioSel = $("cfg-radio");
    var uartSel = $("cfg-uart-baud");
    var usbSel = $("cfg-baud");
    if (!hint || !radioSel) { return; }
    var usbVal = (usbSel && usbSel.value) || "115200";
    var uartVal = (uartSel && uartSel.value) || "9600";
    if (radioSel.value === "e28-2g4m27s") {
      hint.textContent =
        "E28-2G4M27S uses SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI, PD14=NSS @ 4 MHz) + PG10=BUSY, PG11=NRESET, PF13=DIO1, PG12=RX_EN, PG13=TX_EN at 3.3 V ONLY (500 mW PA). Applies live to Vehicle + Ground (USB " +
        usbVal + " Bd, fallback UART " + uartVal + " Bd) and persists to STM32 flash.";
    } else {
      hint.textContent =
        "DX-LR42-24T12D uses USART6 (PG14=TX, PG9=RX @ " + uartVal +
        " 8N1) + PG12=M0, PG13=M1, PG10=AUX. Applies live over AT mode to Vehicle + Ground (USB " +
        usbVal + " Bd) and persists to STM32 flash.";
    }
  }

  function populateBaudSelect(selectEl, list, selectedValue, defaultValue) {
    if (!selectEl) { return; }
    selectEl.textContent = "";
    list.forEach(function (b) {
      var opt = document.createElement("option");
      opt.value = String(b);
      opt.textContent = b + " baud" + (b === defaultValue ? " (default)" : "");
      if (b === selectedValue) { opt.selected = true; }
      selectEl.appendChild(opt);
    });
  }

  function openSettings() {
    if (snapshot && snapshot.config) { config = snapshot.config; }
    var cfg = config || {};
    $("cfg-mode").value = (cfg.op_mode === "test") ? "test" : "live";
    $("cfg-fill").value = cfg.fill_seconds || 60;
    $("cfg-countdown").value = cfg.countdown_seconds || 5;
    updateModeHint();
    $("cfg-confirm").checked = !!cfg.confirm_ign;
    $("cfg-sound").checked = !!cfg.sound;
    populateBaudSelect($("cfg-baud"), USB_BAUDS, cfg.baud || 115200, 115200);
    populateBaudSelect($("cfg-uart-baud"), UART_BAUDS, cfg.uart_baud || 9600, 9600);
    $("cfg-radio").value = radioIdOf(cfg);
    updateRadioHint();
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

    document.querySelectorAll(".control-grid button[data-action], .countdown-banner button[data-action], .secondary-row button[data-action]").forEach(function (btn) {
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
    $("pill-opmode").addEventListener("click", openSettings);
    $("pill-radio").addEventListener("click", openSettings);
    $("cfg-mode").addEventListener("change", updateModeHint);
    $("cfg-countdown").addEventListener("input", updateModeHint);
    $("cfg-radio").addEventListener("change", updateRadioHint);
    $("cfg-baud").addEventListener("change", updateRadioHint);
    $("cfg-uart-baud").addEventListener("change", updateRadioHint);
    $("btn-settings-close").addEventListener("click", function () { stopRecording(); $("settings").close(); });
    $("settings").addEventListener("close", stopRecording);
    $("btn-refresh-ports").addEventListener("click", refreshPorts);

    $("settings-form").addEventListener("submit", function (ev) {
      ev.preventDefault();
      var payload = {
        op_mode: $("cfg-mode").value,
        fill_seconds: parseInt($("cfg-fill").value, 10),
        countdown_seconds: parseInt($("cfg-countdown").value, 10),
        port: $("cfg-port").value,
        baud: parseInt($("cfg-baud").value, 10),
        uart_baud: parseInt($("cfg-uart-baud").value, 10),
        radio: $("cfg-radio").value,
        confirm_ign: $("cfg-confirm").checked,
        sound: $("cfg-sound").checked
      };
      api("/api/config", payload).then(function () {
        toast("Settings applied — updating boards & saving to flash", "ok");
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
    if (isLiveMode() && (action === "srv_toggle" || action === "ign_toggle")) { return; }
    if (!isLiveMode() && action === "srv_ign_toggle") { return; }
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
      if (snapshot) {
        renderFill(snapshot);
        renderCountdown(snapshot);
      }
    }, 50);
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
