// server.js
const WebSocket = require("ws");
const express = require("express");
const app = express();
const PORT = 3000;

// Serve static files (your dashboard)
app.use(express.static(__dirname));
app.listen(PORT, () => console.log(`Server running at http://localhost:${PORT}`));

// WebSocket server
const wss = new WebSocket.Server({ port: 3001 }); // Use a separate port for WS
console.log("WebSocket running on ws://localhost:3001");

wss.on("connection", (ws) => {
  console.log("New client connected");

  ws.on("message", (message) => {
    // Convert to string just in case
    const str = message.toString();

    // Broadcast to all clients
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(str); 
      }
    });
  });

  ws.on("close", () => console.log("Client disconnected"));
});