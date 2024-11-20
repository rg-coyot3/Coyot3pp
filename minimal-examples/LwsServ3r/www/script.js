let content = document.getElementById("content");
content.innerHTML="initialization<br>";

console.log(`connecting to ws://${window.location.host}:${window.location.port}`)
const socket = new WebSocket(`ws://${window.location.host}`)
let interval = null;
socket.onopen = event => {
  console.log(`websocket up`);
  if(interval !== null)
    clearInterval(interval);
  interval = setInterval( () => {
    console.log(`sending message`);
    socket.send("hello world! " + new Date() + "\r\n");
  }, 1000);
}
socket.onmessage = event => {
  console.log(`received message : ${event.data}`);
  content.innerHTML+=`${event.data}<br>`;
}
socket.onclose = event => {
  console.log(`socket closed`);
  clearInterval(interval);
  interval = null;
}
