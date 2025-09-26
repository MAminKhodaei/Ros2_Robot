document.addEventListener('DOMContentLoaded', () => {
    console.log("رابط کاربری بارگذاری شد.");
    
    // --- گرفتن دسترسی به تمام المان‌های نمایشی ---
    const connectionStatusEl = document.getElementById('connection-status');
    const gpsStatusEl = document.getElementById('gps-status');
    const distanceStatusEl = document.getElementById('distance-status');
    const videoStreamEl = document.getElementById('video-stream'); // <-- جدید

    // --- راه‌اندازی نقشه ---
    const map = L.map('map').setView([35.6892, 51.3890], 5);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap'
    }).addTo(map);
    let robotMarker = L.marker([35.6892, 51.3890]).addTo(map).bindPopup('موقعیت ربات');
    let isFirstGpsUpdate = true;

    // Listen for distance updates from the server
    // --- اتصال WebSocket و مدیریت رویدادها ---
    const socket = io();

    socket.on('connect', () => {
        connectionStatusEl.textContent = 'متصل';
        connectionStatusEl.style.color = '#66ff66';
    });

    socket.on('disconnect', () => {
        connectionStatusEl.textContent = 'قطع';
        connectionStatusEl.style.color = '#ff6666';
    });

    socket.on('gps_update', (data) => {
        const { lat, lon } = data;
        const newLatLng = [lat, lon];
        robotMarker.setLatLng(newLatLng);
        gpsStatusEl.textContent = `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
        if (isFirstGpsUpdate) {
            map.setView(newLatLng, 16);
            isFirstGpsUpdate = false;
        }
    });

    socket.on('distance_update', (data) => {
        distanceStatusEl.textContent = data.distance;
    });

    // --- رویداد جدید برای به‌روزرسانی ویدیو ---
    socket.on('video_update', (image_data) => {
        // به‌روز کردن منبع تگ تصویر با داده‌های جدید
        videoStreamEl.src = `data:image/jpeg;base64,${image_data}`;
    });
    // ----------------------------------------

    // --- منطق دکمه‌های کنترل (بدون تغییر) ---
    const buttons_control = {
        'btn-forward': 'forward', 'btn-backward': 'backward',
        'btn-left': 'left', 'btn-right': 'right',
    };
    
    function sendCommand_control(command) {
        socket.emit('control', { command: command });
    }

    for (const [id, command] of Object.entries(buttons_control)) {
        const button = document.getElementById(id);
        if (button) {
            button.addEventListener('mousedown', () => sendCommand_control(command));
            button.addEventListener('mouseup', () => sendCommand_control('stop'));
            button.addEventListener('mouseleave', () => sendCommand_control('stop'));
        }
    }
    document.getElementById('btn-stop').addEventListener('click', () => sendCommand_control('stop'));
});