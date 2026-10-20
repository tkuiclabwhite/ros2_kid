
function detectSftpHost() {
    var path = decodeURIComponent(window.location.pathname || '');
    var m = path.match(/sftp:host=([^,/]+)/);
    return m ? m[1] : null;
}

window.ROBOT_CONFIG = {
    ip: window.location.hostname || detectSftpHost() || '192.168.1.58'
};

// 之後新增/修改已知網路位址，只要改這個清單即可，不用動下面的邏輯
window.ROBOT_KNOWN_ADDRESSES = [
    { label: '有線網路',        ip: '192.168.1.58' },
    { label: '機器人熱點',      ip: '192.168.10.58' },
];

document.addEventListener('DOMContentLoaded', function () {
    var sel = document.getElementById('addressSelect');
    if (!sel) return;

    // 先把目前自動偵測到的位址插在最前面，預設選中它
    var detectedOpt = document.createElement('option');
    detectedOpt.value = window.ROBOT_CONFIG.ip;
    detectedOpt.text  = window.ROBOT_CONFIG.ip + ' (目前)';
    sel.insertBefore(detectedOpt, sel.firstChild);

    // 再把其他已知位址列進去，跳過跟目前偵測到的重複的那一個
    window.ROBOT_KNOWN_ADDRESSES.forEach(function (entry) {
        if (entry.ip === window.ROBOT_CONFIG.ip) return;
        var opt = document.createElement('option');
        opt.value = entry.ip;
        opt.text  = entry.label + ' (' + entry.ip + ')';
        sel.appendChild(opt);
    });

    sel.value = window.ROBOT_CONFIG.ip;
});

