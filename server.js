// server.js
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', ws => {
  console.log('Client connected');

  // Example: send dummy FSR data every second
  const interval = setInterval(() => {
    ws.send("FSR value: " + Math.floor(Math.random() * 100));
  }, 1000);

  ws.on('close', () => {
    clearInterval(interval);
    console.log('Client disconnected');
  });
});

console.log("WebSocket server running on port 8080");