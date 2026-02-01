<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Paddle Controller</title>

<style>
body {
  font-family: Arial;
  text-align: center;
  margin: 5px;
}
button {
  width: 150px;
  height: 30px;
  font-size: 16px;
  padding: 5px;
  cursor: pointer;
  border-radius: 5px;
  color: white;
  border: none;
  background: #007bff;
  margin-bottom: 4px;
}
button:hover { background: #0056b3; }
button:disabled { background: #ccc; cursor: not-allowed; }
/* Small buttons */
.small-button {
  width: 70px;
  height: 30px;
  font-size: 16px;
  padding: 5px;
  cursor: pointer;
  border-radius: 5px;
  color: white;
  border: none;
  background: #007bff;
  margin-bottom: 4px;
}
.columns { display: flex; justify-content: center; gap: 20px; margin-top: 5px; }
.column { display: flex; flex-direction: column; align-items: center; }
.header-container h1 { margin: 5px 0; font-size: 1.5em; }
.header-container { display: flex; flex-direction: column; align-items: center; gap: 5px; }
/* ---- SLIDERS ---- */
.slider-container { display: flex; justify-content: center; gap: 40px; margin-top: 10px; }
.slider-column { display: flex; flex-direction: column; gap: 20px; align-items: center; }
.slider-box { display: flex; flex-direction: column; align-items: center; width: 120px; }
input[type="range"] { writing-mode: bt-lr; -webkit-appearance: slider-vertical; width: 40px; height: 120px; }
.slider-inner { display: flex; align-items: center; gap: 1px; }
.slider-current-value { min-width: 35px; font-weight: bold; text-align: right; }
/* Set Min/Max buttons row */
.button-row { display: flex; gap: 10px; margin-top: 1px; }
.button-row button { flex: 0 0 auto; }
</style>
</head>

<body>

<div class="header-container">
  <h1>Paddle Controller</h1>
  <button id="connect-btn">Connect</button>
</div>

<div class="columns">
  <div class="column">
    <button id="fsr-btn" disabled>Grip Unk</button>
    <div id="fsr-value">FSR: -- Step: --</div>
  </div>
  <div class="column">
    <button id="xangle-btn" disabled>Angle Unk</button>
    <div id="xangle-value">X Angle: --</div>
  </div>
</div>

<div class="slider-container">
  <div class="slider-column">
    <div class="slider-box">
      <label for="grip-start-slider">HAPTIC START</label>
      <div class="slider-inner">
        <div class="slider-current-value" id="grip-start-current">4</div>
        <input type="range" id="grip-start-slider" min="2" max="10" step="1" value="4">
      </div>
    </div>

    <div class="slider-box">
      <label for="haptic-start-pct">HAPTIC %</label>
      <div class="slider-inner">
        <div class="slider-current-value" id="haptic-start-current">50</div>
        <input type="range" id="haptic-start-pct" min="0" max="100" step="1" value="50">
      </div>
    </div>

    <div class="button-row">
       <button id="set-min-btn" class="small-button">Set Min</button>
       <button id="set-max-btn" class="small-button">Set Max</button>
    </div>
  </div>

  <div class="slider-column">
    <div class="slider-box">
      <label for="angle-threshold">ANGLE</label>
      <div class="slider-inner">
        <div class="slider-current-value" id="angle-threshold-current">3</div>
        <input type="range" id="angle-threshold" min="-10" max="10" step="1" value="3">
      </div>
    </div>

    <div class="slider-box">
      <label for="angle-delay">ANGLE DELAY</label>
      <div class="slider-inner">
        <div class="slider-current-value" id="angle-delay-current">0.8</div>
        <input type="range" id="angle-delay" min="0" max="5" step="0.1" value="0.8">
      </div>
    </div>
  </div>
</div>

<script>
let device, server, uartService, txCharacteristic, rxCharacteristic;
let isConnected = false;

let currentFSR = '--';
let currentStep = '--';

const connectBtn = document.getElementById('connect-btn');
const fsrBtn = document.getElementById('fsr-btn');
const fsrValue = document.getElementById('fsr-value');
const xangleBtn = document.getElementById('xangle-btn');
const xangleValue = document.getElementById('xangle-value');

const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const TX_CHAR_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const RX_CHAR_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

// Connect/Disconnect
connectBtn.addEventListener('click', async () => {
  if (!isConnected) {
    try {
      device = await navigator.bluetooth.requestDevice({ filters: [{ services: [SERVICE_UUID] }] });
      server = await device.gatt.connect();
      uartService = await server.getPrimaryService(SERVICE_UUID);
      txCharacteristic = await uartService.getCharacteristic(TX_CHAR_UUID);
      rxCharacteristic = await uartService.getCharacteristic(RX_CHAR_UUID);
      await rxCharacteristic.startNotifications();
      rxCharacteristic.addEventListener('characteristicvaluechanged', handleNotifications);
      isConnected = true;
      connectBtn.textContent = 'Disconnect';
      fsrBtn.disabled = false;
      xangleBtn.disabled = false;
      sendCommand('GET_FSR\n');
      sendCommand('GET_XANGLE\n');
    } catch (err) {
      console.error(err);
      alert('Failed to connect');
    }
  } else {
    if (device?.gatt?.connected) device.gatt.disconnect();
    isConnected = false;
    connectBtn.textContent = 'Connect';
    fsrBtn.disabled = true;
    xangleBtn.disabled = true;
    fsrValue.textContent = 'FSR: -- Step: --';
    xangleValue.textContent = 'X Angle: --';
  }
});

// TOGGLES
fsrBtn.addEventListener('click', () => sendCommand('TOGGLE_FSR\n'));
xangleBtn.addEventListener('click', () => sendCommand('TOGGLE_XANGLE\n'));

function sendCommand(cmd) {
  if (isConnected && txCharacteristic) {
    txCharacteristic.writeValue(new TextEncoder().encode(cmd)).catch(console.error);
  }
}

// BLE RX HANDLER
function handleNotifications(event) {
  const msg = new TextDecoder().decode(event.target.value).trim();
  console.log('RX:', msg);

  if (msg.includes('FSR ENABLED')) fsrBtn.textContent = 'Turn Grip Off';
  else if (msg.includes('FSR DISABLED')) fsrBtn.textContent = 'Turn Grip On';
  else if (msg.startsWith('STEP:')) {
    currentStep = parseInt(msg.split(':')[1].trim());
    fsrValue.textContent = `FSR: ${currentFSR} Step: ${currentStep}`;
  } 
  else if (msg.startsWith('STEP1:')) {
    currentStep = parseInt(msg.split(':')[1].trim());
    fsrValue.textContent = `FSR: ${currentFSR} Step: ${currentStep}`;
    // Re-enable Min button
    if (setMinBtn.disabled) {
      setMinBtn.disabled = false;
      setMinBtn.style.background = '#007bff';
    }
  }
  else if (msg.startsWith('FSR:')) {
    const fsrVal = parseInt(msg.split(':')[1].trim());
    if (!isNaN(currentStep) && fsrVal < 2 * currentStep) {
      console.log('FSR value ignored: below 2x Step1');
    } else {
      currentFSR = fsrVal;
      fsrValue.textContent = `FSR: ${currentFSR} Step: ${currentStep}`;
    }
    // Always re-enable Max button
    if (setMaxBtn.disabled) {
      setMaxBtn.disabled = false;
      setMaxBtn.style.background = '#007bff';
    }
  }
  else if (msg.includes('XANGLE ENABLED')) xangleBtn.textContent = 'Turn Angle Off';
  else if (msg.includes('XANGLE DISABLED')) xangleBtn.textContent = 'Turn Angle On';
  else if (msg.startsWith('XANGLE:')) xangleValue.textContent = `X Angle: ${msg.split(':')[1].trim()}`;
}

// -------------------------
// MIN/MAX BUTTONS
// -------------------------
const setMinBtn = document.getElementById('set-min-btn');
const setMaxBtn = document.getElementById('set-max-btn');

setMinBtn.addEventListener('click', () => {
  if (!isConnected) return;
  setMinBtn.disabled = true;
  setMinBtn.style.background = '#ccc';
  sendCommand('SET_MIN\n');
});

setMaxBtn.addEventListener('click', () => {
  if (!isConnected) return;
  setMaxBtn.disabled = true;
  setMaxBtn.style.background = '#ccc';
  sendCommand('SET_MAX\n');
});

// SLIDER DISPLAY
[
  ['grip-start-slider','grip-start-current'],
  ['haptic-start-pct','haptic-start-current'],
  ['angle-threshold','angle-threshold-current'],
  ['angle-delay','angle-delay-current']
].forEach(([sliderId, displayId]) => {
  const slider = document.getElementById(sliderId);
  const display = document.getElementById(displayId);
  slider.addEventListener('input', () => display.textContent = slider.value);
});
</script>

</body>
</html>
