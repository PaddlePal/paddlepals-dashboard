const WebSocket = require("ws");
const express = require("express");
const http = require("http");

const app = express();
app.use(express.static(__dirname));

// Single server for both HTTP and WebSocket
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

server.listen(3000, () => {
  console.log("Server running at http://localhost:3000");
  console.log("WebSocket running on ws://localhost:3000");
});

wss.on("connection", (ws) => {
  console.log("New client connected");

  ws.on("message", (message) => {
    const str = message.toString();
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(str);
      }
    });
  });

  ws.on("close", () => console.log("Client disconnected"));
});