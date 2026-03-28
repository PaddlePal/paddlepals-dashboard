let socket;

function connectWS() {
  const laptopIP = "172.16.181.150"; // replace with your laptop's IP
  const port = 8080;

  socket = new WebSocket(`ws://${laptopIP}:${port}`);

  socket.onopen = () => {
    document.getElementById("status").innerText = "Connected";
    console.log("Connected to server");
  };

  socket.onmessage = (event) => {
    const dataDiv = document.getElementById("data");
    dataDiv.innerText = event.data + "\n" + dataDiv.innerText;
  };

  socket.onclose = () => {
    document.getElementById("status").innerText = "Disconnected";
    console.log("Disconnected from server");
  };

  socket.onerror = (err) => {
    document.getElementById("status").innerText = "Error";
    console.error("WebSocket error:", err);
  };
}

function sendCommand(cmd) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(cmd);
    console.log("Sent command:", cmd);
  } else {
    alert("WebSocket not connected!");
  }
}