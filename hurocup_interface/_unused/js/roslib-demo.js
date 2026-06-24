/**
 * roslib-demo.js
 * Mock ROSLIB for demo mode.
 * Activates when: URL has ?demo  OR  window._DEMO_MODE_FORCED === true
 * (demo/ copies set _DEMO_MODE_FORCED so ?demo is not needed in URL)
 */
(function () {
    if (!new URLSearchParams(window.location.search).has('demo') && !window._DEMO_MODE_FORCED) return;

    /* ─── Demo data ──────────────────────────────────────────────── */
    var DEMO = {
        walking: {
            com_y_swing: 15, width_size: 35, period_t: 360,
            t_dsp: 40, lift_height: 2.5, stand_height: 23.5, com_height: 29.5
        },
        lc_walking: {
            period_t: 400, com_y_swing: 10, width_size: 35, t_dsp: 40,
            lift_height: 4, stand_height: 23.5, com_height: 29.5,
            board_high: 30, clearance: 3.0, hip_roll: 0, ankle_roll: 0
        },
        hsv: {
            orange: { hmin: 5,  hmax: 25,  smin: 120, smax: 255, vmin: 120, vmax: 255 },
            yellow: { hmin: 20, hmax: 40,  smin: 100, smax: 255, vmin: 100, vmax: 255 },
            blue:   { hmin:100, hmax: 130, smin: 100, smax: 255, vmin:  50, vmax: 255 },
            red:    { hmin:  0, hmax:  10, smin: 100, smax: 255, vmin: 100, vmax: 255 },
            green:  { hmin: 40, hmax:  80, smin:  80, smax: 255, vmin:  80, vmax: 255 },
            white:  { hmin:  0, hmax: 180, smin:   0, smax:  50, vmin: 180, vmax: 255 }
        },
        camera: {
            brightness: 1, contrast: 100, saturation: 50,
            white_balance: 4000, auto_white_balance: false,
            auto_exposure: false, zoomin: 1.0
        }
    };

    /* ─── Topic registry ─────────────────────────────────────────── */
    var _topics = {};   // name → MockTopic

    /* ─── MockRos ────────────────────────────────────────────────── */
    function MockRos(options) {
        this._handlers = {};
        this.socket = { readyState: 3 };   // CLOSED
        var self = this;
        setTimeout(function () {
            self.socket = { readyState: 1 };
            self._emit('connection');
        }, 600);
    }
    MockRos.prototype.on = function (event, cb) {
        if (!this._handlers[event]) this._handlers[event] = [];
        this._handlers[event].push(cb);
    };
    MockRos.prototype.once = function (event, cb) {
        var self = this, wrapper = function () {
            cb.apply(null, arguments);
            self._handlers[event] = (self._handlers[event] || []).filter(function (h) { return h !== wrapper; });
        };
        this.on(event, wrapper);
    };
    MockRos.prototype._emit = function (event, data) {
        var hs = this._handlers[event] || [];
        hs.slice().forEach(function (cb) { cb(data); });
    };
    MockRos.prototype.connect = function () {
        var self = this;
        this.socket = { readyState: 0 };
        setTimeout(function () {
            self.socket = { readyState: 1 };
            self._emit('connection');
        }, 400);
    };
    MockRos.prototype.close = function () {
        var self = this;
        this.socket = { readyState: 3 };
        setTimeout(function () { self._emit('close'); }, 100);
    };

    /* ─── MockTopic ──────────────────────────────────────────────── */
    function MockTopic(options) {
        this.name = options.name;
        this._subs = [];
        _topics[this.name] = this;
    }
    MockTopic.prototype.subscribe = function (cb) { this._subs.push(cb); };
    MockTopic.prototype.publish = function (msg) {
        console.log('[DEMO] publish →', this.name, msg);
    };
    MockTopic.prototype.unsubscribe = function () { this._subs = []; };

    /* ─── MockService ────────────────────────────────────────────── */
    function MockService(options) { this.name = options.name; }
    MockService.prototype.callService = function (req, ok, fail) {
        var name = this.name;
        console.log('[DEMO] service →', name, req);
        setTimeout(function () {
            var res = _handleService(name, req);
            if (res !== null) ok(res);
            else if (fail) fail('DEMO: no mock for ' + name);
        }, 80);
    };

    function _handleService(name, req) {
        switch (name) {
            case '/web/LoadWalkingGaitParameter':
                return Object.assign({}, DEMO.walking);
            case '/web/LoadLCWalkingGaitParameter':
                return Object.assign({}, DEMO.lc_walking);
            case '/LoadHSVInfo': {
                var lbl = (req && req.colorlabel) ? req.colorlabel.toLowerCase() : 'orange';
                return Object.assign({}, DEMO.hsv[lbl] || DEMO.hsv.orange);
            }
            case '/CameraInfo':
                return Object.assign({}, DEMO.camera);
            case '/package/InterfaceCheckSector':
                return { checkflag: true };
            case '/package/InterfaceReadSaveMotion':
                return { vectorcnt: 0, motionstate: [], id: [], motionlist: [],
                         relativedata: [], absolutedata: [], item_names: [], readcheck: true };
            case '/SaveHSV':
            case '/BuildModel':
                return { success: true };
            default:
                return {};
        }
    }

    /* ─── Fake data streams (called after connection fires) ──────── */
    function _pushFakeData() {
        // Interface.js sets url(./picture/...) which is relative to the HTML document.
        // From _static/ that path is wrong — rewrite to ../hurocup_interface/picture/.
        var bg = document.body.style.backgroundImage;
        if (bg && bg.indexOf('./picture/') !== -1) {
            document.body.style.backgroundImage = bg.replace('./picture/', '../hurocup_interface/picture/');
        }
        // Joint states (22 motors, all at 2048 = centre)
        var jt = _topics['/joint_states'];
        if (jt && jt._subs.length) {
            var names = [], pos = [];
            for (var i = 1; i <= 22; i++) { names.push(String(i)); pos.push(2048); }
            jt._subs.slice().forEach(function (cb) { cb({ name: names, position: pos }); });
        }
        // IMU
        var st = _topics['/package/sensorpackage'];
        if (st && st._subs.length) {
            st._subs.slice().forEach(function (cb) { cb({ roll: 0, pitch: 0, yaw: 0 }); });
        }
        // Strategy location (WalkingInterface / miniDRC)
        var lt = _topics['/locationBack'];
        if (lt && lt._subs.length) {
            lt._subs.slice().forEach(function (cb) { cb({ data: 'demo' }); });
        }
        // Camera image placeholder
        var img = document.getElementById('orign_image');
        if (img) {
            // miniDRC uses transform:scale(3.5) for real stream; reset to natural size in demo
            img.style.transform = 'none';
            img.style.marginTop = '0';
            img.style.maxWidth = '100%';
            img.style.height = 'auto';
            img.src = 'data:image/svg+xml;utf8,' + encodeURIComponent(
                '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480">' +
                '<rect width="640" height="480" fill="#1a1a2e"/>' +
                '<text x="320" y="220" font-family="monospace" font-size="22" fill="#03e9f4" text-anchor="middle">[DEMO] Camera Feed</text>' +
                '<text x="320" y="260" font-family="monospace" font-size="14" fill="#888" text-anchor="middle">Connect to robot to see live stream</text>' +
                '</svg>'
            );
        }
        // NodeMonitor: start periodic fake /rosout log stream
        _startFakeRosout();
    }
    setTimeout(_pushFakeData, 900);

    /* ─── Fake /rosout log stream for NodeMonitor ────────────────── */
    var _rosoutTimer = null;
    var _FAKE_LOGS = [
        { name: 'API',              level: 20, msg: '[DEMO] API node running' },
        { name: 'walking_strategy', level: 20, msg: '[DEMO] walking_strategy: step=500 theta=0' },
        { name: 'image_node',       level: 20, msg: '[DEMO] image_node: frame received' },
        { name: 'dynamixel_driver', level: 20, msg: '[DEMO] driver: all motors OK' },
        { name: 'unified_sensor_node', level: 20, msg: '[DEMO] IMU roll=0.0 pitch=0.1 yaw=0.0' },
        { name: 'motion_strategy',  level: 20, msg: '[DEMO] motion_strategy: standby' },
        { name: 'rosbridge_websocket', level: 20, msg: '[DEMO] rosbridge: client connected' },
        { name: 'API',              level: 30, msg: '[DEMO] WARN: ball not found in frame' },
        { name: 'image_node',       level: 20, msg: '[DEMO] image_node: HSV model loaded' },
        { name: 'walking_strategy', level: 20, msg: '[DEMO] walking_strategy: cmd x=0 y=0' }
    ];
    var _rosoutIdx = 0;
    function _startFakeRosout() {
        var topic = _topics['/rosout'];
        if (!topic || !topic._subs.length) return;
        if (_rosoutTimer) return;
        _rosoutTimer = setInterval(function () {
            var topic = _topics['/rosout'];
            if (!topic || !topic._subs.length) { clearInterval(_rosoutTimer); return; }
            var entry = _FAKE_LOGS[_rosoutIdx % _FAKE_LOGS.length];
            _rosoutIdx++;
            topic._subs.slice().forEach(function (cb) { cb(entry); });
        }, 800);
    }

    /* ─── Demo banner ────────────────────────────────────────────── */
    document.addEventListener('DOMContentLoaded', function () {
        var b = document.createElement('div');
        b.id = 'demo-banner';
        b.style.cssText = [
            'position:fixed', 'top:0', 'left:0', 'right:0', 'z-index:99999',
            'background:#ff6b35', 'color:#fff', 'text-align:center',
            'padding:6px 12px', 'font-weight:bold', 'font-size:13px',
            'letter-spacing:1px', 'box-shadow:0 2px 8px rgba(0,0,0,.5)'
        ].join(';');
        b.textContent = 'DEMO 模式 — 模擬資料，無需連接機器人';
        document.body.insertBefore(b, document.body.firstChild);
        // Push demo banner below fixed header if any
        var banner_h = b.offsetHeight;
        document.body.style.paddingTop = (parseInt(document.body.style.paddingTop || 0) + banner_h) + 'px';
    });

    /* ─── Override window.ROSLIB ─────────────────────────────────── */
    window.ROSLIB = {
        Ros:            MockRos,
        Topic:          MockTopic,
        Service:        MockService,
        ServiceRequest: function (v) { if (v) Object.assign(this, v); },
        Message:        function (v) { if (v) Object.assign(this, v); },
        Param:          function () { this.get = function (cb) { cb(null); }; }
    };

    console.log('[roslib-demo] Demo mode active. ROSLIB overridden.');
})();
