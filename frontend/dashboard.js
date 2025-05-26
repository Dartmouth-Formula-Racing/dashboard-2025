const ws = new WebSocket("ws://localhost:9000", "can-protocol");
    ws.onmessage = (event) => {
      // console.log(event);
      const data = JSON.parse(event.data);
      document.getElementById("speed").textContent = data.speed;
      document.getElementById("battery").textContent = data.battery;
      document.getElementById("throttle").textContent = data.throttle;
    };