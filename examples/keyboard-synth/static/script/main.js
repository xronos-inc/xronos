import { WebSocketClient } from './wsClient.js';

let ws = null;
let uriInput = document.getElementById('ws-uri');
let statusIndicator = document.getElementById('status-indicator');
let connectBtn = document.getElementById('connect-btn');
function connectWebSocket(uriText) {
  const uri = uriText.startsWith("ws://") || uriText.startsWith("wss://")
    ? uriText
    : `ws://${uriText}`;
  if (ws) ws.close?.();
  ws = new WebSocketClient(uri, setStatus);
}
connectBtn.onclick = () => { connectWebSocket(uriInput.value); };
function setStatus(connected) {
  statusIndicator.style.background = connected ? 'limegreen' : '#aaa';
}
window.onload = () => {
  const params = new URLSearchParams(window.location.search);
  const uriParam = params.get('ws');

  const host = window.location.hostname;
  const port = parseInt(window.location.port || "8000", 10) + 1;
  const defaultUri = `${host}:${port}`;

  const uriText = uriParam || uriInput.value.trim() || defaultUri;
  uriInput.value = uriText;
  connectWebSocket(uriText);

  renderKeys();
};

const KEY_MAP = {
  "a": "C4", "w": "C#4", "s": "D4", "e": "D#4", "d": "E4", "f": "F4",
  "t": "F#4", "g": "G4", "y": "G#4", "h": "A4", "u": "A#4", "j": "B4", "k": "C5"
};

const ROWS = [
  { keys: ['w', 'e', '', 't', 'y', 'u'], className: 'row top-row' },
  { keys: ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k'], className: 'row bottom-row' }
];

const activeKeys = new Set();

function renderKeys() {
  const container = document.getElementById('keyboard');
  container.innerHTML = '';

  ROWS.forEach(({ keys, className }) => {
    const row = document.createElement('div');
    row.className = className;

    keys.forEach(key => {
      const btn = document.createElement('button');
      if (key) {
        btn.innerHTML = `${key}<br><small>${KEY_MAP[key] || ''}</small>`;
        if (activeKeys.has(key)) btn.classList.add('active');
        btn.onmousedown = () => handleInput(key, 'down');
        btn.onmouseup = () => handleInput(key, 'up');
        btn.onmouseleave = () => {
          if (activeKeys.has(key)) handleInput(key, 'up');
        };
      } else {
        btn.disabled = true;
        btn.style.visibility = 'hidden';
      }
      row.appendChild(btn);
    });

    container.appendChild(row);
  });
}

function handleInput(key, type) {
  if (!(key in KEY_MAP)) return;
  if (type === 'down' && activeKeys.has(key)) return;

  if (type === 'down') activeKeys.add(key);
  else activeKeys.delete(key);

  ws.send({ type, key, note: KEY_MAP[key] });
  renderKeys();
}

document.addEventListener('keydown', e => handleInput(e.key.toLowerCase(), 'down'));
document.addEventListener('keyup', e => handleInput(e.key.toLowerCase(), 'up'));

document.getElementById('exit').onclick = () => ws.send({ type: 'exit' });
