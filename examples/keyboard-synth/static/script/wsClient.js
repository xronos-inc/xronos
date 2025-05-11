export class WebSocketClient {
  constructor(url, setStatus) {
    console.log(`Connecting to WebSocket: ${url}`);
    this.socket = new WebSocket(url);

    this.socket.onopen = () => {
      console.log("WebSocket connected");
      setStatus(true);
    };

    this.socket.onclose = () => {
      console.log("WebSocket disconnected");
      setStatus(false);
    };

    this.socket.onerror = (err) => {
      console.error("WebSocket error:", err);
      setStatus(false);
    };
  }

  send(event) {
    if (this.socket.readyState === WebSocket.OPEN) {
      console.log("Sending event:", event);
      this.socket.send(JSON.stringify(event));
    } else {
      console.warn("WebSocket not open. Cannot send:", event);
    }
  }

  close() {
    console.log("Closing WebSocket");
    this.socket.close();
  }
}