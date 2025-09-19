document.addEventListener('DOMContentLoaded', () => {
    console.log("رابط کاربری بارگذاری شد.");
    
    // --- راه‌اندازی نقشه ---
    const map = L.map('map').setView([35.6892, 51.3890], 5); // مختصات پیش‌فرض (تهران)
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '© OpenStreetMap'
    }).addTo(map);
    let robotMarker = L.marker([35.6892, 51.3890]).addTo(map)
        .bindPopup('موقعیت ربات').openPopup();
    let isFirstGpsUpdate = true;

    // --- اتصال WebSocket ---
    const socket = io();
    const connectionStatusEl = document.getElementById('connection-status');
    const gpsStatusEl = document.getElementById('gps-status');

    socket.on('connect', () => {
        console.log("به سرور متصل شدیم!");
        connectionStatusEl.textContent = 'متصل';
        connectionStatusEl.style.color = '#66ff66';
    });

    socket.on('disconnect', () => {
        console.log("ارتباط با سرور قطع شد!");
        connectionStatusEl.textContent = 'قطع';
        connectionStatusEl.style.color = '#ff6666';
    });

    // جدید: گوش دادن به رویداد gps_update
    socket.on('gps_update', (data) => {
        const { lat, lon } = data;
        console.log(`مختصات GPS دریافت شد: ${lat}, ${lon}`);
        
        const newLatLng = [lat, lon];
        robotMarker.setLatLng(newLatLng);
        gpsStatusEl.textContent = `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
        
        // اگر اولین آپدیت بود، نقشه را روی موقعیت ربات متمرکز کن
        if (isFirstGpsUpdate) {
            map.setView(newLatLng, 16);
            isFirstGpsUpdate = false;
        }
    });

    // --- منطق دکمه‌های کنترل ---
    const buttons = { /* ... (این بخش بدون تغییر باقی می‌ماند) ... */ };
    function sendCommand(command) { /* ... (این بخش بدون تغییر باقی می‌ماند) ... */ }
    // ... (بقیه کدهای مربوط به دکمه‌ها) ...
    
    // کدهای مربوط به دکمه‌ها از پاسخ قبلی را اینجا کپی کنید
    const buttons_control = {
        'btn-forward': 'forward',
        'btn-backward': 'backward',
        'btn-left': 'left',
        'btn-right': 'right',
    };
    
    function sendCommand_control(command) {
        console.log(`ارسال دستور: ${command}`);
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