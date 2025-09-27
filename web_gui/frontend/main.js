document.addEventListener('DOMContentLoaded', () => {
    // --- گرفتن دسترسی به المان‌های HTML ---
    const videoStreamEl = document.getElementById('video-stream');
    const connectionStatusEl = document.getElementById('connection-status');
    const connectionIndicator = document.getElementById('connection-indicator');
    const gpsStatusEl = document.getElementById('gps-status');
    const gpsIndicator = document.getElementById('gps-indicator');
    const distanceStatusEl = document.getElementById('distance-status');
    const distanceIndicator = document.getElementById('distance-indicator');
    const localCameraBtn = document.getElementById('btn-local-camera');
    const localVideoPreview = document.getElementById('local-video-preview');

    // --- راه‌اندازی نقشه ---
    const map = L.map('map').setView([35.6892, 51.3890], 5);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OpenStreetMap' }).addTo(map);
    let robotMarker = L.marker([35.6892, 51.3890]).addTo(map).bindPopup('موقعیت ربات');
    let isFirstGpsUpdate = true;

    // --- اتصال WebSocket ---
    const socket = io();

    // --- مدیریت رویدادهای WebSocket ---
    socket.on('connect', () => {
        connectionStatusEl.textContent = 'متصل';
        connectionIndicator.classList.remove('offline');
        connectionIndicator.classList.add('online');
    });
    socket.on('disconnect', () => {
        connectionStatusEl.textContent = 'قطع';
        connectionIndicator.classList.remove('online');
        connectionIndicator.classList.add('offline');
    });
    
    socket.on('gps_update', (data) => {
        const newLatLng = [data.lat, data.lon];
        robotMarker.setLatLng(newLatLng);
        gpsStatusEl.textContent = `${data.lat.toFixed(5)}, ${data.lon.toFixed(5)}`;
        gpsIndicator.classList.remove('offline');
        gpsIndicator.classList.add('online');
        if (isFirstGpsUpdate) { map.setView(newLatLng, 16); isFirstGpsUpdate = false; }
    });
    socket.on('distance_update', (data) => {
        distanceStatusEl.textContent = `${data.distance} cm`;
        distanceIndicator.classList.remove('offline');
        distanceIndicator.classList.add('online');
    });
    socket.on('video_update', (image_data) => {
        videoStreamEl.src = `data:image/jpeg;base64,${image_data}`;
    });

    // --- منطق دکمه‌های کنترل حرکت ---
    const buttons_control = { 'btn-forward': 'forward', 'btn-backward': 'backward', 'btn-left': 'left', 'btn-right': 'right' };
    function sendCommand(command) { socket.emit('control', { command: command }); }
    for (const [id, command] of Object.entries(buttons_control)) {
        const button = document.getElementById(id);
        if (button) {
            button.addEventListener('mousedown', () => sendCommand(command));
            button.addEventListener('mouseup', () => sendCommand('stop'));
            button.addEventListener('mouseleave', () => sendCommand('stop'));
        }
    }
    document.getElementById('btn-stop').addEventListener('click', () => sendCommand('stop'));

    // --- منطق WebRTC برای دوربین محلی ---
    let pc = null;
    localCameraBtn.addEventListener('click', async () => {
        if (pc) {
            pc.close(); pc = null;
            if (localVideoPreview.srcObject) {
                localVideoPreview.srcObject.getTracks().forEach(track => track.stop());
            }
            localVideoPreview.style.display = 'none';
            localCameraBtn.textContent = 'فعال‌سازی دوربین محلی';
            localCameraBtn.classList.remove('active');
            return;
        }
        try {
            const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });            localVideoPreview.srcObject = stream;
            localVideoPreview.style.display = 'block';
            localCameraBtn.textContent = 'قطع دوربین محلی';
            localCameraBtn.classList.add('active');
            pc = new RTCPeerConnection();
            stream.getTracks().forEach(track => pc.addTrack(track, stream));
            pc.oniceconnectionstatechange = () => console.log(`ICE Connection State: ${pc.iceConnectionState}`);
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            const response = await fetch('/offer', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
            });
            const answer = await response.json();
            await pc.setRemoteDescription(new RTCSessionDescription(answer));
        } catch (e) {
            alert(`خطا در دسترسی به دوربین: ${e.toString()}`);
        }
    });
});